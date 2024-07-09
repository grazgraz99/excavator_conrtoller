#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist, Point
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
        
        self.csv_path = os.path.join(
            get_package_share_directory('excavator_control'),
            'maps',
            'path_record.csv'
        )

        ##Initialize Parameters:
        self.declare_parameter('lookahead_distance')
        self.declare_parameter('steering_gain')
        self.declare_parameter('stationary_turn_angle')
        self.declare_parameter('simulate_turtlebot')
        self.declare_parameter('odom_topic')
        self.declare_parameter('initial_state',[0,0,0,0])

        self.steering_gain=self.get_parameter('steering_gain').value
        self.simulate_turtlebot=self.get_parameter('simulate_turtlebot').value
        self.stationary_turn_angle=self.get_parameter('stationary_turn_angle').value
        self.look_ahead_distance=self.get_parameter('lookahead_distance').value#1

        self.target_behind_flag=False 
        self.publish_sample_path_flag=True

        #self.path=np.array([[1, 1], [-1, -1], [3, -3], [-4, 4], [5,5], [6,6],[7,7]])
        self.path=np.array([[0,0]])
        #print(np.delete(self.path,0,axis=0))
        #print(f'beginning path: {self.path}')

        self.target_idx=-1

        self.x_pos=0
        self.y_pos=0
        self.qx=0
        self.qy=0
        self.qz=0
        self.qw=0
        self.yaw=0


        self.goal_num=0
        

        self.path_pub=self.create_publisher(Float32MultiArray, "waypoints",10)#setup publisher
        self.cmd_vel_pub=self.create_publisher(Twist,"cmd_vel",10)#setup cmd publisher
        self.target_pub=self.create_publisher(Float32MultiArray,"current_target",10)
        


        odom_topic=self.get_parameter('odom_topic').value
        self.odom_subscriber=self.create_subscription(Odometry,odom_topic,self.odom_callback,10)
        self.emergency_stop_subscriber=self.create_subscription(Bool, 'STOP', self.e_stop_callback, 10)

        current_state=self.get_parameter('initial_state').value


        self.load_pts()#load map
        self.point_index=0
        self.publish_path()
        self.timer = self.create_timer(2, self.publish_path)
        self.path_subscriber=self.create_subscription(Float32MultiArray, "waypoints", self.waypoint_callback, 10)
        
        self.EMERGENCY_STOP_FLAG=True 



    def e_stop_callback(self,msg):
        self.EMERGENCY_STOP_FLAG=msg.data

    def publish_path(self):
        data=Float32MultiArray()
        self.next_x_vals=self.x_values[self.point_index:self.point_index+50]
        self.next_y_vals=self.y_values[self.point_index:self.point_index+50]
        for i, j in zip(self.next_x_vals,self.next_y_vals):
            data.data.append(i)
            data.data.append(j)
        #print(f'data: {data.data}')
        self.path_pub.publish(data)
        self.point_index+=50

        #self.waypoint_callback()


    def waypoint_callback(self, msg):
        data = np.array(msg.data)
        #self.path = data.reshape(-1, 2)
        new_data = data.reshape(-1, 2)
        self.path = np.vstack((self.path, new_data))
        #self.publish_path()
       


    def load_pts(self):
        df=pd.read_csv(self.csv_path, sep=',')
        self.x_values=df['x'].tolist()
        self.y_values=df['y'].tolist()

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

        print(f'path size: {self.path.size}')
        print(f'path: {self.path}')
        if self.path.size<10:
            return
        else:
            self.choose_goal()
            
            self.go_to_goal1()#calculates robot heading and distance from target point
            self.publish_twist()
            self.update_plan()#removes target point from array if it has been reached
       
    def choose_goal(self):
        self.distances = np.sqrt((self.path[:, 0] - self.x_pos) ** 2 + (self.path[:, 1] - self.y_pos) ** 2)
        #print('distances',self.distances)
        
        # Find the point just beyond the look-ahead distance
        self.target_idx = np.where(self.distances >= self.look_ahead_distance)[0]
        ##print(f'target_idx {self.target_idx}')

    
        target_point = self.path[self.target_idx][0]
        ##print(f'target_idx[0] {self.target_idx[0]}')
        ##print(f'target {target_point}')

        self.goal_x=target_point[0]
        self.goal_y=target_point[1]

        #publish current target
        target=Float32MultiArray()
        target.data.append(float(self.goal_x))
        target.data.append(float(self.goal_y))

        self.target_pub.publish(target)

            
    def go_to_goal1(self):

        #Calculate angle error
        angle_to_goal = np.arctan2(self.goal_y - self.y_pos, self.goal_x - self.x_pos)
        self.angle_error=float(angle_to_goal-self.yaw)
        self.angle_error = (self.angle_error + np.pi) % (2 * np.pi) - np.pi

        #update flag that triggers robot to turn only or turn and move
        self.update_flag()

        #Calculate distance error array
        self.distance_error= self.distances[self.target_idx][0]
        ##print(f'distance error {self.distance_error}')
        
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

        if self.EMERGENCY_STOP_FLAG==True:   
                command.linear.x=0.0
                command.angular.z=0.0#make the robot move

        if len(self.distances)<=10:
            command.linear.x=0.0
            command.angular.z=0.0#if there are no more points, stop moving

        self.cmd_vel_pub.publish(command)
        
    def update_flag(self):
        if abs(self.angle_error)>self.stationary_turn_angle:
            self.target_behind_flag=True
        else:
            self.target_behind_flag=False

    def update_plan(self):
        #remove the point from the list once close enough to it
            if self.distance_error<self.look_ahead_distance+.0002:
                #print('UPDATED')
                #self.path=np.delete(self.path,0,axis=0)
                self.path = self.path[self.target_idx[0]:]

                #print(f'path: {self.path}')
         



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
