#!/usr/bin/env python
 
# from this import d
import rospy
import random
import numpy as np
from gazebo_msgs.msg import ModelState
# from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from scipy import linalg as lnr
 
class Driver(object):
    # constructor
    def __init__(self):
        self.time_save = 0
        self.name = 'cylinderRobot'
        self.pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
        rospy.Subscriber('/robot/control', Twist, self.callback_control)
        self.mass = 10
        ## ===================================== Edit bellow =====================================
        ## ==========================================================================   vvvvvvvvvv

        # Define publisher object.  Publish the state of robot to topic "/gazebo/set_model_state"
        #   message type is ModelState
        self.pub = rospy.Publisher(...)
        # Define subscriber object. Subscrip the signal from  topic  "/robot/control" sent by teleop
        #   message type is Twist
        #   set the callback function as self.callback_control
        rospy.Subscriber(...)

        # member variable saving system state.
        self.state = np.zeros([4,1])    
        # state = [ vx; px; vy; py ]
        #   dx/dt = Ax(t) + Bu(t) + w(t), 
        #   cov[w(t), w(t)] = Sigma_w

        # Define matrix of continuous system:  A, B (by numpy).
        self.A = ...
        self.B = ...

        ## ==========================================================================  ^^^^^^^^^^
        ## ===================================== Edit above =====================================
        self.Sigma_w = np.eye(4)*0.00001
 
    def callback_control(self, twist):
        if self.time_save == 0:
            self.time_save = rospy.get_time()
        else:
            dt = rospy.get_time() - self.time_save
            self.time_save = rospy.get_time()
            u = np.zeros([2,1])
            u[0] = twist.linear.x
            u[1] = twist.angular.z
	    if u[0] ==0 and self.state[1,0]==0 and self.state[3,0]==0:
		return 0
	    else:
                self.state = self.forward_dynamics(u, dt)
                self.sendStateMsg()

    def forward_dynamics(self, u, dt):

        Atilde, Btilde, Sigma_w_tilde = self._discretization_Func(dt)

        w = np.random.multivariate_normal(np.zeros([4]), Sigma_w_tilde).reshape([4, 1])

        x = Atilde.dot(self.state) + Btilde.dot(u) + w

        return x

    def _discretization_Func(self, dt):
        ## ===================================== Edit bellow =====================================
        ## ==========================================================================   vvvvvvvvvv


        # Please implementation the discretization function here


        ## ==========================================================================  ^^^^^^^^^^
        ## ===================================== Edit above =====================================
        return Atilde, Btilde, Sigma_w_tilde

    def sendStateMsg(self):
        msg = ModelState()
        msg.model_name = self.name
        msg.pose.position.x = self.state[1]
        msg.pose.position.y = self.state[3]
        self.pub.publish(msg)
 
       
       
 
if __name__ == '__main__':
    try:
        rospy.init_node('driver', anonymous=True)
        driver = Driver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
