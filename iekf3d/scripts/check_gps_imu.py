#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script to subscribe to GPS and IMU messages and publish odometry messages.
This is useful for checking GPS and IMU data quality by visualizing the combined odometry.

The script:
1. Subscribes to GPS (sensor_msgs/NavSatFix) and IMU (sensor_msgs/Imu) messages
2. Converts GPS lat/lon to UTM coordinates
3. Subtracts the map's UTM origin (from iekf3d config)
4. Combines GPS position with IMU orientation
5. Publishes as nav_msgs/Odometry

Usage:
    rosrun iekf3d check_gps_imu.py --gps-topic /gps/fix --imu-topic /imu/data
"""

import rospy
import argparse
import numpy as np
from pathlib import Path
import yaml
from threading import Lock

# ROS message types
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Quaternion
import tf2_ros
from geometry_msgs.msg import TransformStamped

# For GPS conversion
try:
    from geodesy import utm
    gps_converter_available = True
except ImportError:
    gps_converter_available = False
    rospy.logerr("geodesy not found. GPS conversion will not work.")
    rospy.logerr("Install with: sudo apt-get install ros-noetic-geodesy")


def load_map_utm_origin(config_path=None):
    """
    Load the UTM origin from the map's info.yaml file.
    
    Args:
        config_path: Path to iekf3d configs.yaml file. If None, uses default path.
        
    Returns:
        tuple: (utm_origin_x, utm_origin_y, utm_origin_z) or (0, 0, 0) if not found
    """
    try:
        # If no config path provided, use default
        if config_path is None:
            script_dir = Path(__file__).resolve().parent.parent
            config_path = script_dir / "configs" / "configs.yaml"
        else:
            config_path = Path(config_path)
        
        if not config_path.exists():
            rospy.logwarn(f"Config file not found: {config_path}")
            return (0.0, 0.0, 0.0)
        
        # Load iekf3d config
        with open(config_path, 'r') as f:
            iekf_config = yaml.safe_load(f)
        
        map_path = iekf_config.get('map_path')
        if not map_path:
            rospy.logwarn("map_path not found in config")
            return (0.0, 0.0, 0.0)
        
        # Handle relative paths
        map_path = Path(map_path)
        if not map_path.is_absolute():
            map_path = config_path.parent / map_path
        
        # Load map info.yaml
        info_path = map_path / "info.yaml"
        if not info_path.exists():
            rospy.logwarn(f"Map info file not found: {info_path}")
            return (0.0, 0.0, 0.0)
        
        with open(info_path, 'r') as f:
            map_info = yaml.safe_load(f)
        
        utm_origin = map_info.get('utm_origin', [0.0, 0.0, 0.0])
        
        if len(utm_origin) >= 3:
            rospy.loginfo(f"Loaded UTM origin from {info_path}")
            rospy.loginfo(f"UTM origin: [{utm_origin[0]:.3f}, {utm_origin[1]:.3f}, {utm_origin[2]:.3f}]")
            return (utm_origin[0], utm_origin[1], utm_origin[2])
        else:
            rospy.logwarn("Invalid utm_origin in map info")
            return (0.0, 0.0, 0.0)
            
    except Exception as e:
        rospy.logerr(f"Error loading UTM origin: {e}")
        return (0.0, 0.0, 0.0)


def load_imu_extrinsic(config_path=None):
    """
    Load IMU extrinsic rotation from config.
    
    Args:
        config_path: Path to iekf3d configs.yaml file.
        
    Returns:
        numpy.ndarray: 3x3 rotation matrix from IMU to body frame, or identity if not found
    """
    try:
        if config_path is None:
            script_dir = Path(__file__).resolve().parent.parent
            config_path = script_dir / "configs" / "configs.yaml"
        else:
            config_path = Path(config_path)
        
        if not config_path.exists():
            rospy.logwarn(f"Config file not found: {config_path}")
            return np.eye(3)
        
        with open(config_path, 'r') as f:
            iekf_config = yaml.safe_load(f)
        
        imu_extinct = iekf_config.get('imu_extinct', [])
        
        if len(imu_extinct) >= 3:
            # imu_extinct are rotation angles in radians: [rx, ry, rz]
            # Tbi = Rx * Ry * Rz
            rx, ry, rz = imu_extinct[0], imu_extinct[1], imu_extinct[2]
            
            # Rotation matrices
            Rx = np.array([[1, 0, 0],
                          [0, np.cos(rx), -np.sin(rx)],
                          [0, np.sin(rx), np.cos(rx)]])
            
            Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                          [0, 1, 0],
                          [-np.sin(ry), 0, np.cos(ry)]])
            
            Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                          [np.sin(rz), np.cos(rz), 0],
                          [0, 0, 1]])
            
            Tbi = Rx @ Ry @ Rz
            rospy.loginfo(f"Loaded IMU extrinsic: rx={np.degrees(rx):.2f}°, ry={np.degrees(ry):.2f}°, rz={np.degrees(rz):.2f}°")
            return Tbi
        else:
            rospy.loginfo("No IMU extrinsic found, using identity")
            return np.eye(3)
            
    except Exception as e:
        rospy.logerr(f"Error loading IMU extrinsic: {e}")
        return np.eye(3)


class GPSIMUOdometry:
    """Combines GPS and IMU data to publish odometry."""
    
    def __init__(self, gps_topic, imu_topic, odom_topic="/gps_imu_odometry", 
                 config_path=None, publish_tf=False):
        """
        Initialize the GPS-IMU odometry publisher.
        
        Args:
            gps_topic: Topic name for GPS messages
            imu_topic: Topic name for IMU messages
            odom_topic: Topic name for output odometry
            config_path: Path to iekf3d config file
            publish_tf: Whether to publish TF transform
        """
        self.gps_topic = gps_topic
        self.imu_topic = imu_topic
        self.odom_topic = odom_topic
        self.publish_tf = publish_tf
        
        # Load UTM origin and IMU extrinsic
        self.utm_origin = load_map_utm_origin(config_path)
        self.imu_extrinsic = load_imu_extrinsic(config_path)
        
        # State variables
        self.lock = Lock()
        self.last_gps_position = None
        self.last_gps_time = None
        self.last_imu_orientation = None
        self.last_imu_time = None
        
        # Publishers
        self.odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=10)
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Subscribers
        self.gps_sub = rospy.Subscriber(gps_topic, NavSatFix, self.gps_callback, queue_size=10)
        self.imu_sub = rospy.Subscriber(imu_topic, Imu, self.imu_callback, queue_size=10)
        
        rospy.loginfo(f"GPS-IMU Odometry Node Initialized")
        rospy.loginfo(f"  GPS topic: {gps_topic}")
        rospy.loginfo(f"  IMU topic: {imu_topic}")
        rospy.loginfo(f"  Odometry topic: {odom_topic}")
        rospy.loginfo(f"  Publish TF: {publish_tf}")
    
    def gps_callback(self, msg):
        """Callback for GPS messages."""
        if not gps_converter_available:
            rospy.logwarn_throttle(10, "geodesy not available. Cannot convert GPS to UTM.")
            return
        
        if msg.status.status < 0:
            rospy.logwarn_throttle(10, "GPS fix not available.")
            return
        
        try:
            # Convert lat/lon to UTM
            utm_point = utm.fromLatLong(msg.latitude, msg.longitude, msg.altitude)
            
            # Subtract map's UTM origin to get coordinates relative to map origin
            x = utm_point.easting - self.utm_origin[0]
            y = utm_point.northing - self.utm_origin[1]
            z = utm_point.altitude - self.utm_origin[2]
            
            with self.lock:
                self.last_gps_position = (x, y, z)
                self.last_gps_time = msg.header.stamp
            
            # Publish odometry if we have both GPS and IMU
            self.publish_odometry(msg.header.stamp)
            
        except Exception as e:
            rospy.logerr(f"GPS conversion failed: {e}")
    
    def imu_callback(self, msg):
        """Callback for IMU messages."""
        try:
            # Extract quaternion from IMU
            q_imu = np.array([msg.orientation.x, msg.orientation.y, 
                             msg.orientation.z, msg.orientation.w])
            
            # Apply IMU extrinsic calibration: pose.so3() = Tbi.matrix() * q
            # Convert quaternion to rotation matrix
            q = np.array([msg.orientation.w, msg.orientation.x, 
                         msg.orientation.y, msg.orientation.z])
            R_imu = self.quaternion_to_rotation_matrix(q)
            
            # Apply extrinsic calibration
            R_body = self.imu_extrinsic @ R_imu
            
            # Convert back to quaternion
            q_body = self.rotation_matrix_to_quaternion(R_body)
            
            with self.lock:
                self.last_imu_orientation = q_body
                self.last_imu_time = msg.header.stamp
            
            # Publish odometry if we have both GPS and IMU
            self.publish_odometry(msg.header.stamp)
            
        except Exception as e:
            rospy.logerr(f"IMU processing failed: {e}")
    
    def quaternion_to_rotation_matrix(self, q):
        """
        Convert quaternion [w, x, y, z] to 3x3 rotation matrix.
        """
        w, x, y, z = q
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
        ])
    
    def rotation_matrix_to_quaternion(self, R):
        """
        Convert 3x3 rotation matrix to quaternion [w, x, y, z].
        """
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        # Normalize
        norm = np.sqrt(w*w + x*x + y*y + z*z)
        return np.array([w/norm, x/norm, y/norm, z/norm])
    
    def publish_odometry(self, stamp):
        """Publish odometry message combining GPS position and IMU orientation."""
        with self.lock:
            if self.last_gps_position is None or self.last_imu_orientation is None:
                return
            
            position = self.last_gps_position
            orientation = self.last_imu_orientation
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = "iekf_map"
        odom_msg.child_frame_id = "gps_imu_base_link"
        
        # Set position from GPS
        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = position[2]
        
        # Set orientation from IMU (with extrinsic applied)
        odom_msg.pose.pose.orientation.w = orientation[0]
        odom_msg.pose.pose.orientation.x = orientation[1]
        odom_msg.pose.pose.orientation.y = orientation[2]
        odom_msg.pose.pose.orientation.z = orientation[3]
        
        # Publish odometry
        self.odom_pub.publish(odom_msg)
        
        # Publish TF if enabled
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = "iekf_map"
            tf_msg.child_frame_id = "gps_imu_base_link"
            tf_msg.transform.translation.x = position[0]
            tf_msg.transform.translation.y = position[1]
            tf_msg.transform.translation.z = position[2]
            tf_msg.transform.rotation.w = orientation[0]
            tf_msg.transform.rotation.x = orientation[1]
            tf_msg.transform.rotation.y = orientation[2]
            tf_msg.transform.rotation.z = orientation[3]
            self.tf_broadcaster.sendTransform(tf_msg)


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Combine GPS and IMU messages to publish odometry',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage
  rosrun iekf3d check_gps_imu.py --gps-topic /gps/fix --imu-topic /imu/data
  
  # With custom output topic
  rosrun iekf3d check_gps_imu.py --gps-topic /gps/fix --imu-topic /imu/data --odom-topic /my_odometry
  
  # Publish TF transform
  rosrun iekf3d check_gps_imu.py --gps-topic /gps/fix --imu-topic /imu/data --publish-tf
  
  # Use custom config file
  rosrun iekf3d check_gps_imu.py --gps-topic /gps/fix --imu-topic /imu/data --config-path /path/to/configs.yaml
        """
    )
    
    parser.add_argument('--gps-topic', type=str, default='/Inertial/gps/fix',
                        help='ROS topic for GPS messages (sensor_msgs/NavSatFix)')
    parser.add_argument('--imu-topic', type=str, default='/Inertial/imu/data',
                        help='ROS topic for IMU messages (sensor_msgs/Imu)')
    parser.add_argument('--odom-topic', type=str, default='/gps_imu_odometry',
                        help='ROS topic for output odometry (default: /gps_imu_odometry)')
    parser.add_argument('--config-path', type=str, default=None,
                        help='Path to iekf3d configs.yaml file (default: iekf3d/configs/configs.yaml)')
    parser.add_argument('--publish-tf', action='store_true',
                        help='Publish TF transform from iekf_map to gps_imu_base_link')
    
    args = parser.parse_args()
    
    # Initialize ROS node
    rospy.init_node('gps_imu_odometry', anonymous=True)
    rospy.loginfo("GPS-IMU Odometry Node Started")
    
    # Create GPS-IMU odometry publisher
    gps_imu_odom = GPSIMUOdometry(
        gps_topic=args.gps_topic,
        imu_topic=args.imu_topic,
        odom_topic=args.odom_topic,
        config_path=args.config_path,
        publish_tf=args.publish_tf
    )
    
    rospy.loginfo("Node running... Press Ctrl+C to stop")
    
    # Spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")


if __name__ == '__main__':
    main()
