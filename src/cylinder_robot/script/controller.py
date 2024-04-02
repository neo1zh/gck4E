#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState

# read points
def read_points():
    path = ''
    x = []
    y = []
    with open(path, 'r') as f:
        for line in f:
            x.append(float(line.split()[0]))
            y.append(float(line.split()[1]))
    return x, y

# def controller():
#     pub = rospy.Publisher('/robot/control', Twist, queue_size = 10)
    
#     rospy.init_node('talker',anonymous=True)
#     #============design your trace below=============
#     rate = rospy.Rate(10)
#     for i in range(0,100):
#         twist = Twist()
#         twist.linear.x=1.5*abs(i-49.5)/(i-49.5)
#         #twist.angular.z=0.5*abs(i-49.5)/(i-49.5)
#         pub.publish(twist)
#         rate.sleep()
#     sys.exit(0)

class PID_Controller:
    def __init__(self,kp,ki,kd,output_min,output_max):
        # init PID controller
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0
        self.last_error = 0
        self.error_sum = 0
        self.error_diff = 0
        # output limit
        self.output_min = output_min
        self.output_max = output_max
        self.output = 0

    def constrain(self, output):
        # limit the output
        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min
        else:
            output = output
        return output
    
    def reset(self):
        # reset the PID controller
        self.error = 0
        self.last_error = 0
        self.error_sum = 0
        self.error_diff = 0
        self.output = 0

    def get_output(self, error):
        # get the output of PID controller
        self.error = error
        self.error_sum += self.error
        self.error_diff = self.error - self.last_error
        self.last_error = self.error

        output = self.kp * self.error + self.ki * self.error_sum + self.kd * self.error_diff
        self.output = self.constrain(output)

        return self.output

class Controller:
    def __init__(self):
        self.name = 'cylinderRobot'
        rospy.Subscriber('/robot/esti_model_state', ModelState, self.callback_state)
        self.pub = rospy.Publisher("/robot/control", Twist, queue_size=10)
        self.state = ModelState()
        self.pid_x = PID_Controller(1.3, 0, 0.4, -0.6, 0.6)
        self.pid_y = PID_Controller(1.3, 0, 0.4, -0.6, 0.6)
        self.pid_vx = PID_Controller(10, 0, 0.01, -18, 18)
        self.pid_vy = PID_Controller(10, 0, 0.01, -18, 18)
        self.px = 0
        self.py = 0
        self.targets_x, self.targets_y = read_points()
        self.target_index = 0  
        self.target_max_index = len(self.targets_x) - 1 
        self.tolerance = 0.1

    def callback_state(self, modelstate):
        self.state = modelstate
        self.px = modelstate.pose.position.x
        self.py = modelstate.pose.position.y

    def calculate_error(self):
        error_x = self.targets_x[self.target_index] - self.px
        error_y = self.targets_y[self.target_index] - self.py
        return error_x, error_y
    


    def control(self):
        twist = Twist()
        error_x, error_y = self.calculate_error()
        
        if self.target_index == self.target_max_index:
            rospy.loginfo('Finish')
            return

        if abs(error_x) < self.tolerance and abs(error_y) < self.tolerance:

            self.target_index += 1
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_vx.reset()
            self.pid_vy.reset()
            twist.linear.x = 0
            twist.angular.z = 0
            rospy.loginfo('target_index: %d', self.target_index)
        else:
            tmp_x = self.pid_x.get_output(error_x)
            tmp_y = self.pid_y.get_output(error_y)

            error_vx = tmp_x - self.state.twist.linear.x
            error_vy = tmp_y - self.state.twist.angular.z

            force_vx = self.pid_vx.get_output(error_vx)
            force_vy = self.pid_vy.get_output(error_vy)

            twist.linear.x = force_vx
            twist.angular.z = force_vy

        self.pub.publish(twist)
        rospy.loginfo('error_x: %f, error_y: %f, twist.linear.x: %f, twist.angular.z: %f', error_x, error_y, twist.linear.x, twist.angular.z)
        
        

if __name__ == '__main__':
    try:
        rospy.init_node('controller_node')
        controller = Controller()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            controller.control()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
