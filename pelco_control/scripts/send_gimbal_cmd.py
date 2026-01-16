#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
云台命令发布工具
用于通过命令行向 /gimbal_cmd 话题发布 GimbalCommand 消息，控制 pelco_control 节点

支持的命令:
    STOP      - 停止所有运动
    UP        - 向上移动 (data: 速度 0-63)
    DOWN      - 向下移动 (data: 速度 0-63)
    LEFT      - 向左移动 (data: 速度 0-63)
    RIGHT     - 向右移动 (data: 速度 0-63)
    UPLEFT    - 左上移动 (data: 速度 0-63)
    UPRIGHT   - 右上移动 (data: 速度 0-63)
    DOWNLEFT  - 左下移动 (data: 速度 0-63)
    DOWNRIGHT - 右下移动 (data: 速度 0-63)
    PAN       - 水平旋转至指定角度 (data: 角度值)
    TILT      - 俯仰旋转至指定角度 (data: 角度值)
    ASKPAN    - 查询水平位置
    ASKTILT   - 查询俯仰位置

使用示例:
    # 停止云台
    python send_gimbal_cmd.py --cmd stop
    
    # 水平旋转到10度
    python send_gimbal_cmd.py --cmd pan --data 10.0
    
    # 俯仰旋转到20度
    python send_gimbal_cmd.py --cmd tilt --data 20.0
    
    # 向左移动，速度30
    python send_gimbal_cmd.py --cmd left --data 30
    
    # 向上移动，速度20
    python send_gimbal_cmd.py --cmd up --data 20
    
    # 查询当前水平角度
    python send_gimbal_cmd.py --cmd askpan
"""

import rospy
import argparse
import sys
from cyber_msgs.msg import GimbalCommand

# 命令枚举定义 (对应 pelco_node.cpp 中的定义)
GIMBAL_COMMANDS = {
    'stop': 0x00,       # 停止
    'up': 0x08,         # 上
    'down': 0x10,       # 下
    'left': 0x04,       # 左
    'right': 0x02,      # 右
    'upleft': 0x0C,     # 上左
    'upright': 0x0A,    # 上右
    'downleft': 0x14,   # 下左
    'downright': 0x12,  # 下右
    'pan': 0x4B,        # 水平旋转至指定角度 (75)
    'tilt': 0x4D,       # 俯仰旋转至指定角度 (77)
    'askpan': 0x51,     # 查询水平位置 (81)
    'asktilt': 0x53,    # 查询俯仰位置 (83)
}


def send_gimbal_command(cmd_name, data_value):
    """
    发送云台命令到 /gimbal_cmd 话题
    
    Args:
        cmd_name: 命令名称 (字符串，如 'pan', 'tilt', 'stop' 等)
        data_value: 命令数据 (角度或速度值)
    """
    # 检查命令是否有效
    cmd_name_lower = cmd_name.lower()
    if cmd_name_lower not in GIMBAL_COMMANDS:
        rospy.logerr(f"未知命令: {cmd_name}")
        rospy.logerr(f"支持的命令: {', '.join(GIMBAL_COMMANDS.keys())}")
        return False
    
    # 初始化ROS节点
    rospy.init_node('gimbal_cmd_sender', anonymous=True)
    
    # 创建发布器
    pub = rospy.Publisher('/gimbal_cmd', GimbalCommand, queue_size=1)
    
    # 等待发布器准备就绪
    rospy.sleep(0.5)
    
    # 创建消息
    msg = GimbalCommand()
    msg.header.stamp = rospy.Time.now()
    msg.cmd = GIMBAL_COMMANDS[cmd_name_lower]
    msg.data = data_value
    
    # 发布消息
    rospy.loginfo(f"发送云台命令: {cmd_name.upper()} (0x{msg.cmd:02X}), 数据: {data_value}")
    pub.publish(msg)
    
    # 等待消息发送完成
    rospy.sleep(0.2)
    
    rospy.loginfo("命令已发送")
    return True


def main():
    """主函数：解析命令行参数并发送云台命令"""
    parser = argparse.ArgumentParser(
        description='向 /gimbal_cmd 话题发布云台控制命令',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  %(prog)s --cmd stop                    # 停止云台
  %(prog)s --cmd pan --data 10.0         # 水平旋转到10度
  %(prog)s --cmd tilt --data 20.0        # 俯仰旋转到20度
  %(prog)s --cmd left --data 30          # 向左移动，速度30
  %(prog)s --cmd up --data 20            # 向上移动，速度20
  %(prog)s --cmd askpan                  # 查询水平位置

支持的命令:
  stop, up, down, left, right, upleft, upright, downleft, downright,
  pan, tilt, askpan, asktilt
"""
    )
    
    parser.add_argument(
        '--cmd',
        type=str,
        required=True,
        help='云台命令 (例如: pan, tilt, stop, left, right 等)'
    )
    
    parser.add_argument(
        '--data',
        type=float,
        default=0.0,
        help='命令数据值 (角度值或速度值，默认: 0.0)'
    )
    
    # 解析参数
    args = parser.parse_args()
    
    try:
        # 发送命令
        success = send_gimbal_command(args.cmd, args.data)
        if not success:
            sys.exit(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
