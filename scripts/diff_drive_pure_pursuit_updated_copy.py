#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist, Point, Pose2D
from nav_msgs.msg import Odometry
import numpy as np
from transforms3d.euler import quat2euler
import pandas as pd
import math
import os
from ament_index_python.packages import get_package_share_directory

'''
'''
class state_calc(Node):
    def __init__(self):
        super().__init__('pa1_task1_node')
        

        ##Initialize Parameters:
        self.declare_parameter('lookahead_distance')
        self.declare_parameter('steering_gain')
        self.declare_parameter('stationary_turn_angle')
        self.declare_parameter('simulate_turtlebot')
        self.declare_parameter('odom_topic')
        self.declare_parameter('command_vel_topic')

        self.steering_gain=self.get_parameter('steering_gain').value
        self.simulate_turtlebot=self.get_parameter('simulate_turtlebot').value
        self.stationary_turn_angle=self.get_parameter('stationary_turn_angle').value
        self.look_ahead_distance=self.get_parameter('lookahead_distance').value#1
        self.odom_topic=self.get_parameter('odom_topic').value
        self.cmd_vel_topic=self.get_parameter('command_vel_topic').value


        odom_msg_type=Odometry


        self.target_behind_flag=False 
        self.publish_sample_path_flag=True

        self.x_pos=0
        self.y_pos=0
        self.qx=0
        self.qy=0
        self.qz=0
        self.qw=0
        self.yaw=0
        self.goal_x=0
        self.goal_y=0

        #'cmd_vel' is the actual topic to publish to
        self.cmd_vel_pub=self.create_publisher(Twist,self.cmd_vel_topic,10)#setup cmd publisher        


        self.target_pose_subscriber=self.create_subscription(Pose2D, '/goal_pose', self.goal_pose_callback, 10)
        self.odom_subscriber=self.create_subscription(Odometry,self.odom_topic,self.odom_callback,10)
        self.emergency_stop_subscriber=self.create_subscription(Bool, 'STOP', self.e_stop_callback, 10)

        
        self.EMERGENCY_STOP_FLAG=True 


    def goal_pose_callback(self, data):
        self.goal_x=data.x
        self.goal_y=data.y
        self.goal_theta=data.theta

    def e_stop_callback(self,msg):
        self.EMERGENCY_STOP_FLAG=msg.data


    def odom_callback(self, data):

        self.x_pos=data.pose.pose.position.x
        self.y_pos=data.pose.pose.position.y
        #print(f'x: {self.x_pos}, y: {self.y_pos}')

        #bot orientation(quaternion form)
        self.qx=data.pose.pose.orientation.x
        self.qy=data.pose.pose.orientation.y
        self.qz=data.pose.pose.orientation.z
        self.qw=data.pose.pose.orientation.w
        self.quaternion= [self.qw, self.qx, self.qy, self.qz]

        ##print(f'x_pos: {self.x_pos} y_pos: {self.y_pos}')
        #Calculate heading angle(euler angle form)
        euler=quat2euler(self.quaternion)
        ##print(f'euler {euler}')
        self.yaw=euler[2]
        


        if self.goal_x==None:
            return
        else:
            #calculate distance
            self.distance_error = np.sqrt((self.goal_x - self.x_pos) ** 2 + (self.goal_y - self.y_pos) ** 2)

            #Calculate yaw angle error
            angle_to_goal = np.arctan2(self.goal_y - self.y_pos, self.goal_x - self.x_pos)
            self.angle_error=float(angle_to_goal-self.yaw)
            self.angle_error = (self.angle_error + np.pi) % (2 * np.pi) - np.pi

            #update turn only flag
            self.update_flag()

            #publish 
            self.publish_twist()
        
    def publish_twist(self):
        command=Twist()   
        command.angular.z=self.steering_gain*self.angle_error #set angular velocity
        ##print(f'angle error {self.angle_error}')


        if self.distance_error>1:
            self.distance_error=1.0
 

        self.update_flag()

        if self.target_behind_flag==True:
            #command.linear.x=0.0
            if self.EMERGENCY_STOP_FLAG==True:
                command.linear.x=0.0
                command.angular.z=0.0
        else:
            command.linear.x=1.0#float(4*self.distance_error)
            command.angular.z=0.0 #This is the condition that makes the
            #robot only turn or only drive straight

        if self.EMERGENCY_STOP_FLAG==True or self.distance_error<.5:
                command.linear.x=0.0
                command.angular.z=0.0#make the robot move

        # if len(self.distances)<1:
        # # if not self.distances:
        #     command.linear.x=0.0
        #     command.angular.z=0.0#if there are no more points, stop moving

        self.cmd_vel_pub.publish(command)
        
    def update_flag(self):
        if abs(self.angle_error)>self.stationary_turn_angle:
            self.target_behind_flag=True
        else:
            self.target_behind_flag=False

def main(args=None):
    rclpy.init(args=args)
    state_calc_node=state_calc()
    
    try:
        rclpy.spin(state_calc_node)
    except KeyboardInterrupt:
        command=Twist()
        command.linear.x=0.0
        command.angular.z=0.0
        state_calc_node.cmd_vel_pub.publish(command)
    state_calc_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
