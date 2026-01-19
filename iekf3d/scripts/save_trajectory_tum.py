#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script to convert ROS odometry/GPS messages to TUM format and save to file.
TUM format: timestamp x y z qx qy qz qw

Usage:
    rosrun iekf3d save_trajectory_tum.py --odom-topic /odometry/filtered --gps-topic /gps/fix
"""

import rospy
import argparse
import os
from datetime import datetime
from pathlib import Path
import threading

# ROS message types
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from cyber_msgs.msg import LocalizationEstimate

# For GPS conversion
try:
    from geodesy import utm
    gps_converter_available = True
except ImportError:
    gps_converter_available = False
    rospy.logwarn("geodesy not found. GPS conversion may not work properly.")
    rospy.logwarn("Install with: sudo apt-get install ros-noetic-geodesy")


class TrajectoryRecorder:
    """Records trajectory data from ROS topics and saves in TUM format."""
    
    def __init__(self, odom_topic=None, gps_topic=None, output_dir=None, 
                 odom_type='nav_msgs/Odometry', gps_type='sensor_msgs/NavSatFix'):
        """
        Initialize the trajectory recorder.
        
        Args:
            odom_topic: Topic name for odometry messages
            gps_topic: Topic name for GPS messages
            output_dir: Directory to save output files
            odom_type: Message type for odometry ('nav_msgs/Odometry', 'geometry_msgs/PoseWithCovarianceStamped', 'cyber_msgs/LocalizationEstimate')
            gps_type: Message type for GPS ('sensor_msgs/NavSatFix', 'cyber_msgs/LocalizationEstimate')
        """
        self.odom_topic = odom_topic
        self.gps_topic = gps_topic
        self.odom_type = odom_type
        self.gps_type = gps_type
        
        # Setup output directory
        if output_dir is None:
            # Default to iekf3d/results/<timestamp>
            script_dir = Path(__file__).resolve().parent.parent
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.output_dir = script_dir / "results" / timestamp
        else:
            self.output_dir = Path(output_dir)
        
        self.output_dir.mkdir(parents=True, exist_ok=True)
        rospy.loginfo(f"Output directory: {self.output_dir}")
        
        # Initialize file handles
        self.odom_file = None
        self.gps_file = None
        self.lock = threading.Lock()
        
        # Open output files
        if self.odom_topic:
            odom_filename = self.output_dir / "trajectory_odom.txt"
            self.odom_file = open(odom_filename, 'w')
            self.odom_file.write("# TUM format trajectory from odometry\n")
            self.odom_file.write("# timestamp x y z qx qy qz qw\n")
            rospy.loginfo(f"Writing odometry to: {odom_filename}")
            
        if self.gps_topic:
            gps_filename = self.output_dir / "trajectory_gps.txt"
            self.gps_file = open(gps_filename, 'w')
            self.gps_file.write("# TUM format trajectory from GPS\n")
            self.gps_file.write("# timestamp x y z qx qy qz qw\n")
            rospy.loginfo(f"Writing GPS to: {gps_filename}")
        
        # Message counters
        self.odom_count = 0
        self.gps_count = 0
        
        # Setup subscribers
        self.subscribers = []
        if self.odom_topic:
            self._setup_odom_subscriber()
        if self.gps_topic:
            self._setup_gps_subscriber()
    
    def _setup_odom_subscriber(self):
        """Setup subscriber for odometry messages based on message type."""
        if self.odom_type == 'nav_msgs/Odometry':
            sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback)
        elif self.odom_type == 'geometry_msgs/PoseWithCovarianceStamped':
            sub = rospy.Subscriber(self.odom_topic, PoseWithCovarianceStamped, self._pose_cov_callback)
        elif self.odom_type == 'cyber_msgs/LocalizationEstimate':
            sub = rospy.Subscriber(self.odom_topic, LocalizationEstimate, self._localization_callback)
        else:
            rospy.logerr(f"Unsupported odometry message type: {self.odom_type}")
            return
        
        self.subscribers.append(sub)
        rospy.loginfo(f"Subscribed to {self.odom_topic} [{self.odom_type}]")
    
    def _setup_gps_subscriber(self):
        """Setup subscriber for GPS messages based on message type."""
        if self.gps_type == 'sensor_msgs/NavSatFix':
            sub = rospy.Subscriber(self.gps_topic, NavSatFix, self._gps_callback)
        elif self.gps_type == 'cyber_msgs/LocalizationEstimate':
            sub = rospy.Subscriber(self.gps_topic, LocalizationEstimate, self._gps_localization_callback)
        else:
            rospy.logerr(f"Unsupported GPS message type: {self.gps_type}")
            return
        
        self.subscribers.append(sub)
        rospy.loginfo(f"Subscribed to {self.gps_topic} [{self.gps_type}]")
    
    def _odom_callback(self, msg):
        """Callback for nav_msgs/Odometry messages."""
        timestamp = msg.header.stamp.to_sec()
        pose = msg.pose.pose
        
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        
        self._write_tum_line(self.odom_file, timestamp, x, y, z, qx, qy, qz, qw)
        self.odom_count += 1
        
        if self.odom_count % 100 == 0:
            rospy.loginfo(f"Recorded {self.odom_count} odometry messages")
    
    def _pose_cov_callback(self, msg):
        """Callback for geometry_msgs/PoseWithCovarianceStamped messages."""
        timestamp = msg.header.stamp.to_sec()
        pose = msg.pose.pose
        
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        
        self._write_tum_line(self.odom_file, timestamp, x, y, z, qx, qy, qz, qw)
        self.odom_count += 1
        
        if self.odom_count % 100 == 0:
            rospy.loginfo(f"Recorded {self.odom_count} pose messages")
    
    def _localization_callback(self, msg):
        """Callback for cyber_msgs/LocalizationEstimate messages (as odometry)."""
        timestamp = msg.header.stamp.to_sec()
        pose = msg.pose
        
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        
        self._write_tum_line(self.odom_file, timestamp, x, y, z, qx, qy, qz, qw)
        self.odom_count += 1
        
        if self.odom_count % 100 == 0:
            rospy.loginfo(f"Recorded {self.odom_count} localization messages")
    
    def _gps_callback(self, msg):
        """Callback for sensor_msgs/NavSatFix messages."""
        # Convert GPS to UTM coordinates
        if not gps_converter_available:
            rospy.logwarn_throttle(10, "geodesy not available. Cannot convert GPS to UTM.")
            return

        if msg.status.status != 2:
            rospy.logwarn_throttle(10, "GPS fix not available.")
            return

        timestamp = msg.header.stamp.to_sec()
        
        # Convert lat/lon to UTM using geodesy
        try:
            utm_point = utm.fromLatLong(msg.latitude, msg.longitude, msg.altitude)
            utm_x = utm_point.easting
            utm_y = utm_point.northing
            z = utm_point.altitude
        except Exception as e:
            rospy.logerr(f"GPS conversion failed: {e}")
            return
        
        # GPS messages don't have orientation, use identity quaternion
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        
        self._write_tum_line(self.gps_file, timestamp, utm_x, utm_y, z, qx, qy, qz, qw)
        self.gps_count += 1
        
        if self.gps_count % 100 == 0:
            rospy.loginfo(f"Recorded {self.gps_count} GPS messages")
    
    def _gps_localization_callback(self, msg):
        """Callback for cyber_msgs/LocalizationEstimate messages (as GPS)."""
        timestamp = msg.header.stamp.to_sec()
        pose = msg.pose
        
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        
        self._write_tum_line(self.gps_file, timestamp, x, y, z, qx, qy, qz, qw)
        self.gps_count += 1
        
        if self.gps_count % 100 == 0:
            rospy.loginfo(f"Recorded {self.gps_count} GPS localization messages")
    
    def _write_tum_line(self, file_handle, timestamp, x, y, z, qx, qy, qz, qw):
        """Write a line in TUM format to file."""
        if file_handle is None:
            return
        
        with self.lock:
            line = f"{timestamp:.6f} {x:.6f} {y:.6f} {z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n"
            file_handle.write(line)
            file_handle.flush()
    
    def close(self):
        """Close all file handles."""
        if self.odom_file:
            self.odom_file.close()
            rospy.loginfo(f"Saved {self.odom_count} odometry messages")
        
        if self.gps_file:
            self.gps_file.close()
            rospy.loginfo(f"Saved {self.gps_count} GPS messages")
        
        rospy.loginfo(f"All files saved to: {self.output_dir}")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Record ROS trajectory messages to TUM format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Record odometry only
  rosrun iekf3d save_trajectory_tum.py --odom-topic /slam/odometry
  
  # Record GPS only
  rosrun iekf3d save_trajectory_tum.py --gps-topic /gps/fix
  
  # Record both odometry and GPS
  rosrun iekf3d save_trajectory_tum.py --odom-topic /slam/odometry --gps-topic /gps/fix
  
  # Specify custom output directory
  rosrun iekf3d save_trajectory_tum.py --odom-topic /slam/odometry --output-dir /tmp/trajectories
  
  # Specify message types
  rosrun iekf3d save_trajectory_tum.py --odom-topic /slam/pose --odom-type geometry_msgs/PoseWithCovarianceStamped
        """
    )
    
    parser.add_argument('--odom-topic', type=str, default=None,
                        help='ROS topic for odometry messages')
    parser.add_argument('--gps-topic', type=str, default=None,
                        help='ROS topic for GPS messages')
    parser.add_argument('--output-dir', type=str, default=None,
                        help='Output directory (default: iekf3d/results/<timestamp>)')
    parser.add_argument('--odom-type', type=str, default='nav_msgs/Odometry',
                        choices=['nav_msgs/Odometry', 
                                'geometry_msgs/PoseWithCovarianceStamped',
                                'cyber_msgs/LocalizationEstimate'],
                        help='Message type for odometry topic')
    parser.add_argument('--gps-type', type=str, default='sensor_msgs/NavSatFix',
                        choices=['sensor_msgs/NavSatFix',
                                'cyber_msgs/LocalizationEstimate'],
                        help='Message type for GPS topic')
    
    args = parser.parse_args()
    
    # Check if at least one topic is specified
    if args.odom_topic is None and args.gps_topic is None:
        parser.error("At least one of --odom-topic or --gps-topic must be specified")
    
    # Initialize ROS node
    rospy.init_node('trajectory_recorder', anonymous=True)
    rospy.loginfo("Trajectory Recorder Node Started")
    
    # Create recorder
    recorder = TrajectoryRecorder(
        odom_topic=args.odom_topic,
        gps_topic=args.gps_topic,
        output_dir=args.output_dir,
        odom_type=args.odom_type,
        gps_type=args.gps_type
    )
    
    # Setup shutdown hook
    rospy.on_shutdown(recorder.close)
    
    rospy.loginfo("Recording... Press Ctrl+C to stop")
    
    # Spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
    finally:
        recorder.close()


if __name__ == '__main__':
    main()
