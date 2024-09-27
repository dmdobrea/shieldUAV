#!/usr/bin/env python
# -*- coding: utf-8 -*-

import 	rclpy
import geometry_msgs.msg

from 	rclpy.node 		import Node
from    rclpy.qos       import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg 	import Twist
from   sensor_msgs.msg  import Image     # Imports the built-in Image message type
import cv2                                  # OpenCV library - import before cv_bridge
from   cv_bridge        import CvBridge  # Package to convert between ROS and OpenCV Images


class MainControl(Node):

	def __init__(self):
		super().__init__('main_control_node')
        
		qos_profile = QoSProfile(
			reliability = QoSReliabilityPolicy.BEST_EFFORT,             
			durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL,          
			history     = QoSHistoryPolicy.KEEP_LAST,                   
			depth       = 1 )

		#================================== Create subscriptions from PX4 autopilot
		# 
		self.img_subscription = self.create_subscription( Image, 'video_detection', self.video_listener_callback, qos_profile)
		self.get_logger().info("Subscription is active")
  
        #================================== Create publisher
        #
		self.velocity_pub     = self.create_publisher (geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)
		self.get_logger().info("The publisher is active")
        
		#================================== Create the TIMER
		# creates callback function for the main command loop 
		timer_period = 0.02  # seconds
		self.timer = self.create_timer(timer_period, self.main_cmdloop_callback)
		self.get_logger().info("The main control timer is active")
		
		self.save_frame = None
		
		# Used to convert images between ROS 2 and OpenCV
		self.br = CvBridge()

	def video_listener_callback(self, newFrame):
		# Convert ROS Image message to OpenCV image
		self.save_frame = self.br.imgmsg_to_cv2(newFrame)
        
	#publishes offboard control modes and velocity as trajectory setpoints
	def main_cmdloop_callback(self):
		# use the save_frame
		
		# !!!!!
		
		# send the coomands to the "velocity_control_node"
		twist = geometry_msgs.msg.Twist()
        
		twist.linear.x  = 0.0
		twist.linear.y  = 0.0
		twist.linear.z  = 0.0
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = 0.0
            
		self.velocity_pub.publish(twist)


def main(args=None):
	
	# Initialize the ROS 2 library for Python
    rclpy.init(args=args)
    
	# Create the node
    main_control = MainControl()

	# Spin the node so the callback function is called.
    rclpy.spin(main_control)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    main_control.destroy_node()
    
    # Shutdown the ROS 2 client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
