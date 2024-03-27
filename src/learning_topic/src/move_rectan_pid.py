#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
x, y, theta = 0, 0, 0 # initial
class PID_Controller:
    def __init__(self,kp,ki,kd,output_min,output_max):
        # 初始化PID的三个参数，以及误差项
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0
        self.last_error = 0
        self.error_sum = 0
        self.error_diff = 0
        # 初始化最大输出与最小输出
        self.output_min = output_min
        self.output_max = output_max
        self.output = 0
    def constrain(self, output):
        # 控制器输出阈值限制
        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min
        else:
            output = output
        return output
    def get_output(self, error):
        # 使用位置式PID获取输出
        self.error = error
        self.error_sum += self.error
        self.error_diff = self.error - self.last_error
        self.last_error = self.error
        # PID的输出
        output = (self.kp*self.error + self.ki*self.error_sum + self.kd*self.error_diff)
        # 限制输出
        self.output = -self.constrain(output)
        return self.output
def PoseCallback(pose):
    # 获取小海龟的初始位置信息，以便后计算相对位移
    global x, y, theta
    x = pose.x - 5.544444561  # 5.544444561 小海龟初始位置
    y = pose.y - 5.544444561
    theta = pose.theta
def velocity_publisher():
    global x, y, theta
    # ROS节点初始化
    rospy.init_node('velocity_publisher', anonymous=True)
    # 新建订阅者subscriber
    rospy.Subscriber('/turtle1/pose', Pose, PoseCallback)
    # 新建发布者publisher
    turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    #设置循环的频率
    rate = rospy.Rate(10)
    #设置小乌龟初始mode
    state = 0
    cmd_vel = Twist()
    # 填入轨迹形状，若为矩形，则如下所示：
    L, H = 3, 2 # 矩形长宽
    
    move_er_threshold = 1e-2
    rotate_er_threshold = 1e-2
    
    # PID控制器部分实例化，PID参数可调
    # 输入合适的PID参数
    move_controller = PID_Controller(0.00001, 0, 0, 0.1, -0.1)  
    rotate_controller = PID_Controller(0.000005, 0, 0, 0.1, -0.1)
    
    while not rospy.is_shutdown():
        if state == 0:
        # mode0: 使得小乌龟前进L距离，画出矩形的一条边
            error = L - x
            if abs(error) < move_er_threshold: #如果到达目标位置（误差小于阈值）
                state = state + 1 #进入下一个状态
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                move_controller.error = 0  # 重置控制器
            else:
                cmd_vel.linear.x = move_controller.get_output(error)
                cmd_vel.angular.z = 0
        elif state == 1:
        # mode1: 使得小乌龟旋转math.pi / 2，画出矩形的直角部分
            error = math.pi / 2 - theta
            if abs(error) < rotate_er_threshold: #如果到达目标位置（误差小于阈值）
                state = state + 1 #进入下一个状态
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                rotate_controller.error = 0
            else:
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = rotate_controller.get_output(error)
        elif state == 2:
        # mode0: 使得小乌龟前进L距离，画出矩形的一条边
            error = H - y
            if abs(error) < move_er_threshold: #如果到达目标位置（误差小于阈值）
                state = state + 1 #进入下一个状态
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                move_controller.error = 0  # 重置控制器
            else:
                cmd_vel.linear.x = move_controller.get_output(error)
                cmd_vel.angular.z = 0
        elif state == 3:
        # mode1: 使得小乌龟旋转math.pi / 2，画出矩形的直角部分
            error = math.pi - theta
            if abs(error) < rotate_er_threshold: #如果到达目标位置（误差小于阈值）
                state = state + 1 #进入下一个状态
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                rotate_controller.error = 0
            else:
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = rotate_controller.get_output(error)
        elif state == 4:
        # mode0: 使得小乌龟前进L距离，画出矩形的一条边
            error = x - 0
            if abs(error) < move_er_threshold: #如果到达目标位置（误差小于阈值）
                state = state + 1 #进入下一个状态
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                move_controller.error = 0  # 重置控制器
            else:
                cmd_vel.linear.x = move_controller.get_output(error)
                cmd_vel.angular.z = 0
        elif state == 5:
        # mode1: 使得小乌龟旋转math.pi / 2，画出矩形的直角部分
            error = - math.pi / 2 - theta
            if abs(error) < rotate_er_threshold: #如果到达目标位置（误差小于阈值）
                state = state + 1 #进入下一个状态
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                rotate_controller.error = 0
            else:
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = rotate_controller.get_output(error)
        elif state == 6:
        # mode0: 使得小乌龟前进L距离，画出矩形的一条边
            error = y - 0
            if abs(error) < move_er_threshold: #如果到达目标位置（误差小于阈值）
                state = state + 1 #进入下一个状态
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                move_controller.error = 0  # 重置控制器
            else:
                cmd_vel.linear.x = move_controller.get_output(error)
                cmd_vel.angular.z = 0
        elif state == 7:
        # mode1: 使得小乌龟旋转math.pi / 2，画出矩形的直角部分
            error = theta - 0
            if abs(error) < rotate_er_threshold: #如果到达目标位置（误差小于阈值）
                state = (state + 1)%8 #进入下一个状态
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
                rotate_controller.error = 0
            else:
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = rotate_controller.get_output(error)

        
        # 发布消息
        turtle_vel_pub.publish(cmd_vel)
        rospy.loginfo("cmd_vel: [%0.2f m/s, %0.2f rad/s] state: %0.1f", cmd_vel.linear.x, cmd_vel.angular.z, state)
        # 按照循环频率延时
        rate.sleep()
if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass