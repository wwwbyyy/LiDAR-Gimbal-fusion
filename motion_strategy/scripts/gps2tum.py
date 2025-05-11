#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix, Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import geodesy.utm

# 全局变量存储 GPS 和 IMU 数据
gps_data = None
imu_data = None

# 输出文件路径
output_file = "gps_truth.txt"

utm_origin = [351425.09269358893, 3433830.3251591502, 15.168673515319824]

# GPS 回调函数
def gps_callback(msg):
    global gps_data
    gps_data = {
        "timestamp": msg.header.stamp.to_sec(),
        "latitude": msg.latitude,
        "longitude": msg.longitude,
        "altitude": msg.altitude
    }
    save_to_tum()

# IMU 回调函数
def imu_callback(msg):
    global imu_data
    imu_data = {
        "timestamp": rospy.Time.now().to_sec(),
        "orientation": msg.orientation
    }
    save_to_tum()

# 保存数据到 TUM 文件
def save_to_tum():
    global gps_data, imu_data

    # 确保 GPS 和 IMU 数据都已接收到
    if gps_data is None or imu_data is None:
        return

    # 提取 GPS 平移信息
    utm_point = geodesy.utm.fromLatLong(gps_data["latitude"], gps_data["longitude"])
    # 将 UTM 坐标转换为相对于原点的坐标
    tx = utm_point.easting - utm_origin[0]
    ty = utm_point.northing - utm_origin[1]
    tz = gps_data["altitude"] - utm_origin[2]

    # 提取 IMU 旋转信息（四元数）
    qx = imu_data["orientation"].x
    qy = imu_data["orientation"].y
    qz = imu_data["orientation"].z
    qw = imu_data["orientation"].w

    # 使用 GPS 时间戳作为记录时间
    timestamp = gps_data["timestamp"]

    # 写入 TUM 文件
    with open(output_file, "a") as f:
        f.write(f"{timestamp} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")

    rospy.loginfo(f"Saved to TUM: {timestamp} {tx} {ty} {tz} {qx} {qy} {qz} {qw}")

def main():
    # 初始化 ROS 节点
    rospy.init_node('gps_imu_to_tum', anonymous=True)

    # 订阅 GPS 和 IMU 数据
    gps_topic = rospy.get_param("~gps_topic", "/Inertial/gps/fix")  # 默认 GPS 话题
    imu_topic = rospy.get_param("~imu_topic", "/Inertial/imu/data")  # 默认 IMU 话题

    rospy.Subscriber(gps_topic, NavSatFix, gps_callback)
    rospy.Subscriber(imu_topic, Imu, imu_callback)

    # 清空或创建 TUM 文件
    with open(output_file, "w") as f:
        f.write("# TUM trajectory file\n")
        f.write("# timestamp tx ty tz qx qy qz qw\n")

    rospy.loginfo("GPS and IMU to TUM node started. Listening for messages...")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass