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

        # member variable saving system state.
        self.state = np.zeros([4,1])    
        # state = [ vx; px; vy; py ]
        #   dx/dt = Ax(t) + Bu(t) + w(t), 
        #   cov[w(t), w(t)] = Sigma_w

        # Define matrix of continuous system:  A, B (by numpy).
        self.A = np.array([[0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1], [0, 0, 0, 0]])
        self.B = np.array([[0, 0], [1/self.mass, 0], [0, 0], [0, 1/self.mass]])

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
            u[1] = twist.linear.y
            if u[0] == 0 and self.state[1,0] == 0 and self.state[3,0] == 0:
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
        Atilde =  np.array([
            [1, 0, 0, 0],
            [dt, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, dt, 1]
        ])

        Btilde = np.array([
            [dt/self.mass, 0],
            [dt**2/(2*self.mass), 0],
            [0, dt/self.mass],
            [0, dt**2/(2*self.mass)]
        ])

        Sigma_w_tilde = np.array([
            [dt*self.Sigma_w[0, 0], dt**2/2*self.Sigma_w[0, 0], 0, 0],
            [dt**2/2*self.Sigma_w[0, 0], dt*self.Sigma_w[1, 1]+dt**3/3*self.Sigma_w[0, 0], 0, 0],
            [0, 0, dt*self.Sigma_w[2, 2], dt**2/2*self.Sigma_w[2, 2]],
            [0, 0, dt**2/2*self.Sigma_w[2, 2], dt*self.Sigma_w[3, 3]+dt**3/3*self.Sigma_w[2, 2]]
        ])
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
