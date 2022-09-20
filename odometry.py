#!/usr/bin/env python

import rospy
import numpy as np
import tf
from std_msgs.msg import Float64MultiArray, Float64
from numpy import pi, sin, cos, arctan2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class rover_odometry():
    def __init__(self):
        
        # wheel radius 
        self.wheel_radius = 0.135
        # width of rover 
        self.vehichle_width = 0.89

        # velocities of wheels
        self.left_vel = np.zeros((2,1), dtype = np.float)
        self.right_vel = np.zeros((2,1), dtype = np.float)

        # starting position of rover. I assume intial pose values all zero
        self.position = np.zeros((3,1), dtype = np.float)
        self.yaw = 0.0

        # last and current time 
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        #self.mes_time = [0, 0]
        self.rate = rospy.Rate(50)

        # linear and angular velocities 
        self.ang_vel = 0.0
        self.lin_vel = 0.0

        rospy.Subscriber('/drive_system_left_motors_feedbacks', Float64MultiArray, self.callback_left)
        rospy.Subscriber('/drive_system_right_motors_feedbacks', Float64MultiArray, self.callback_right)
        rospy.Subscriber('/imu1/data', Imu, self.imu_callback)
        self.pub = rospy.Publisher('/rover/wheel_odom', Odometry, queue_size = 10)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.publisher()
       
    def imu_callback(self, msg):
        
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        roll, pitch, self.yaw = euler_from_quaternion(quaternion)
            
        # callback function calculate left wheels speed
    def callback_left(self, msg):
        left_rpm = np.array(msg.data)
        self.left_vel = self.rpm_to_ms(left_rpm) # converting rpm to m/s 
        
        # callback function calculate right wheels speed
    def callback_right(self, msg):
        right_rpm = np.array(msg.data)
        self.right_vel = self.rpm_to_ms(right_rpm) # converting rpm to m/s 

        # Function to convert rpm to m/s 
    def rpm_to_ms(self, rpm):
        return (2*pi*self.wheel_radius*rpm)/60.0

        # Function to calculate angular and linear velocity of rover
    def velocities(self):
        # Calculate average values of both sides 
        avg_right_vel = (self.right_vel[0]+self.right_vel[1])/2.0
        avg_left_vel = (self.left_vel[0]+self.left_vel[1])/2.0
        
        #self.ang_vel = (avg_right_vel-avg_left_vel)/self.vehichle_width
        self.lin_vel = (avg_right_vel+avg_left_vel)/2.0
        
    def position_calc(self):
        
        self.current_time = rospy.Time.now()
        delta_T = (self.current_time-self.last_time).to_sec()

        self.position[2] = (self.yaw) # Getting yaw directly from imu 
        self.position[0] += self.lin_vel*cos(self.position[2])*delta_T
        self.position[1] += self.lin_vel*sin(self.position[2])*delta_T
        
        self.last_time = self.current_time

    def publisher(self):
        while not rospy.is_shutdown():

            self.velocities()
            self.position_calc()

            odom_quat = quaternion_from_euler(0, 0, self.position[2])
            
            self.odom_broadcaster.sendTransform(
                (self.position[0], self.position[1], 0.0),
                odom_quat,
                self.current_time,
                'base_link',
                'odom'
            )

            odom = Odometry()

            odom.header.stamp = self.current_time
            odom.header.frame_id = 'odom'

            odom.pose.pose = Pose(Point(self.position[0], self.position[1], 0.0), Quaternion(*odom_quat))

            odom.child_frame_id = 'base_link'
            odom.twist.twist = Twist(Vector3(self.lin_vel, 0.0, 0.0), Vector3(0.0, 0.0, self.ang_vel))
            self.pub.publish(odom)
            self.rate.sleep()


if __name__ == '__main__':
    try: 
        rospy.init_node('Odometry_publisher')

        odometry = rover_odometry()
        
    except rospy.ROSInterruptException:
        pass