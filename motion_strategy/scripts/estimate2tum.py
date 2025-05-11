#!/usr/bin/env python3

import rospy
from cyber_msgs.msg import LocalizationEstimate

def localization_callback(msg):
  # 将cyber_msgs/LocalizationEstimate消息转换为tum格式的字符串
  tum_string = convert_to_tum_format(msg)

  # 将tum格式的字符串写入文件
  with open('output_traj.txt', 'a') as file:
    file.write(tum_string)

def convert_to_tum_format(msg):
  # 在这里实现将cyber_msgs/LocalizationEstimate消息转换为tum格式的逻辑
  # 返回tum格式的字符串

  # 示例：假设tum格式的字符串为"x y z qx qy qz qw"
  tum_string = f"{msg.header.stamp.to_sec()} {msg.pose.position.x} {msg.pose.position.y} {msg.pose.position.z} {msg.pose.orientation.x} {msg.pose.orientation.y} {msg.pose.orientation.z} {msg.pose.orientation.w}\n"
  rospy.loginfo(f"Converted to TUM format: {tum_string.strip()}")
  return tum_string

def main():
  rospy.init_node('localization_to_tum_converter')
  rospy.Subscriber('/localization/estimation', LocalizationEstimate, localization_callback)
  rospy.loginfo("Listening to localization_topic...")
  rospy.spin()

if __name__ == '__main__':
  main()