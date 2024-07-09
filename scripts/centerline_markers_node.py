#!/usr/bin/env python3
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node
import rclpy, math
import pandas as pd
import rospkg
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory
import os





'''
Publishes markers for RVIZ showing the centerline that the vehicle is following
'''
class centerline_node(Node):
    def __init__(self):
        super().__init__('center_line_node')

        self.csv_path = os.path.join(
            get_package_share_directory('excavator_control'),
            'maps',
            'path_record.csv'
        )

        self.publisher=self.create_publisher( Marker,'/visualization_marker', 10)
        self.target_marker_publisher=self.create_publisher( Marker,'/target_marker', 10)

        self.target_listener=self.create_subscription(Float32MultiArray, 'current_target',self.current_target_callback, 10)

        self.overall()

    def current_target_callback(self, msg):
        self.marker_msg=Marker()
        self.marker_msg.header.frame_id="odom"
        self.marker_msg.header.stamp=self.get_clock().now().to_msg()

        self.marker_msg.action=0#Marker.ADD
        
        self.marker_msg.type=2#Marker.LINE_STRIP
        self.marker_msg.scale.x=.25
        self.marker_msg.scale.y=.25
        self.marker_msg.scale.z=.25
        self.marker_msg.pose.orientation.w=1.0
        self.marker_msg.color.b=1.0
        self.marker_msg.color.a=1.0

        self.marker_msg.pose.position.x=msg.data[0]
        self.marker_msg.pose.position.y=msg.data[1]

        self.target_marker_publisher.publish(self.marker_msg)

        

    def overall(self):
        self.load_pts()
        self.timer=self.create_timer(1/60,self.marker_publisher)
        #self.marker_publisher()
        pass
        
        
    def load_pts(self):
        df=pd.read_csv(self.csv_path, sep=',')
        self.x_values=df['x'].tolist()
        self.y_values=df['y'].tolist()

        

    def marker_publisher(self):
        self.marker_msg=Marker()
        self.marker_msg.header.frame_id="odom"
        self.marker_msg.header.stamp=self.get_clock().now().to_msg()

        self.marker_msg.action=0#Marker.ADD
        
        self.marker_msg.type=4#Marker.LINE_STRIP
        self.marker_msg.scale.x=.05
        # marker_msg.scale.y=2
        # marker_msg.scale.z=2
        self.marker_msg.pose.orientation.w=1.0
        self.marker_msg.color.g=1.0
        self.marker_msg.color.a=1.0
        for x, y in zip(self.x_values, self.y_values):
            p=Point()
            p.x=x
            p.y=y
            self.marker_msg.points.append(p)
        self.publisher.publish(self.marker_msg)


def main(args=None):
    rclpy.init(args=args)
    Centerline_node=centerline_node()
    rclpy.spin(Centerline_node)
    #centerline_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

        