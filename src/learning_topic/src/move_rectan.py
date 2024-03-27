#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, pi

current_pose = None

# 回调函数，获取当前海龟机器人的位置
def pose_callback(msg):
    global current_pose
    current_pose = msg

# 控制海龟机器人运动画矩形
def move_rect(width, height):
    global current_pose
    # 初始化 ROS 节点
    rospy.init_node('move_rect', anonymous=True)
    # 订阅海龟机器人的位置信息
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    # 创建一个 Publisher，发布速度命令到 /turtle1/cmd_vel
    turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 控制循环频率
    # 等待获取当前位置信息
    while current_pose is None:
        rospy.sleep(0.1)
    # 计算目标位置
    target_x = current_pose.x + width
    target_y = current_pose.y
    # 循环画矩形
    while not rospy.is_shutdown():
        vel_msg = Twist()
        # 计算当前位置和目标位置的方向角
        target_angle = atan2(height, width)
        # 计算当前位置到目标位置的距离
        dist = ((current_pose.x - target_x) ** 2 + (current_pose.y - target_y) ** 2) ** 0.5
        # 如果距离大于设定的阈值，则向目标位置移动
        if dist > 0.1:
            # 计算线速度和角速度
            vel_msg.linear.x = 0.2  # 线速度设为 0.2 m/s
            vel_msg.angular.z = 0.0  # 角速度设为 0 rad/s
        else:
            # 距离小于阈值，到达目标位置，停止运动并旋转
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = pi / 2  # 角速度设为 pi/2 rad/s，即90度/s
            # 更新目标位置和方向
            target_x = current_pose.x + width
            target_y = current_pose.y
        # 发布速度命令
        turtle_vel_pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_rect(3.0, 2.0)  # 画一个宽度为 3 米，高度为 2 米的矩形
    except rospy.ROSInterruptException:
        pass
