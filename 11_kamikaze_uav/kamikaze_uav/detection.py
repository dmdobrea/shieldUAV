#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os

sys.path.append('/usr/local/share/pynq-venv/lib/python3.10/site-packages')
os.environ['PYNQ_JUPYTER_NOTEBOOKS'] = '/root/jupyter_notebooks'
os.environ['BOARD'] = 'KV260'
os.environ['XILINX_XRT'] = '/usr'
os.environ['PATH'] = os.environ.get('PATH') + ':/usr/local/share/pynq-venv/bin/microblazeel-xilinx-elf/bin/'
os.environ['LD_LIBRARY_PATH'] = '/usr/lib'

import rclpy                                		# Python Client Library for ROS 2
from   rclpy.qos           			import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from   rcl_interfaces.msg  			import ParameterDescriptor
from   rclpy.node          			import Node      # Handles the creation of nodes
from   sensor_msgs.msg     			import Image     # Imports the built-in Image message type
import cv2                                  		 # OpenCV library - import before cv_bridge!
from   cv_bridge           			import CvBridge  # Package to convert between ROS and OpenCV Images
from   .submodules.object_detection import ObjCenter

class UAVdetection(Node):
	"""
	Here I create a UAVdetection class, which is a subclass of the Node class.
	"""
	def __init__(self):
		"""
		Class constructor to set up the node
		"""
		# Initiate the Node class's constructor and give it a sugegestive name
		super().__init__('video_detection_node')
        
		qos_profile = QoSProfile(
			reliability = QoSReliabilityPolicy.BEST_EFFORT,                  # RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
			durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL,               # RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
			history     = QoSHistoryPolicy.KEEP_LAST,                        # RMW_QOS_POLICY_HISTORY_KEEP_LAST,
			depth       = 1 )

		# creates a parameter(s) with the name and default value(s)
		verbose_descriptor = ParameterDescriptor(description='Add verbosity to the running node.')
		self.declare_parameter('verbose', 1, verbose_descriptor)

		#get parameters        
		verbose    = self.get_parameter('verbose').get_parameter_value().integer_value

		self.obj = ObjCenter("/home/ubuntu/ros2_kamikazeUAV_ws/src/kamikaze_uav/kamikaze_uav/KR260_yolo3t_UAV_q.h5.xmodel")

		# Create the subscriber. This subscriber will 
		# receive an Image from the video_frames topic
		self.subscription = self.create_subscription( Image, 'video_frames', self.video_detection_callback, qos_profile)

		# Create the publisher. This publisher will publish images 
		# of type Image to the "video_detection" topic.
		self.imagePublisher_ = self.create_publisher( Image, 'video_detection', qos_profile)

		# Used to convert images between ROS 2 and OpenCV
		self.br = CvBridge()

	def video_detection_callback(self, inputFrame):
		"""
		Callback function - when a new frame has arrived, a jump is made here
		"""
        
		verbose = self.get_parameter('verbose').get_parameter_value().integer_value	
        
		# Display the message on the console
		if verbose == 1:
			self.get_logger().info('VD: Received a new video frame')
			
		# Convert ROS Image message to OpenCV image
		input_frame = self.br.imgmsg_to_cv2(inputFrame)
		
		# ==================================================================================
		(H, W) = input_frame.shape[:2]
		centerX = W // 2
		centerY = H // 2
		
		myObjectDet = self.obj.update(input_frame, (centerX, centerY), 0)
		if verbose == 1:
			self.get_logger().info('\nVD: First detection done')
		
		((myObjectX, myObjectY), rect, alg) = myObjectDet		
		
		if alg == 1:
			if rect is not None:
				[top, left, bottom, right] = rect
				cv2.rectangle (input_frame, (int(left), int(top)), (int(right), int(bottom)), (255, 0, 0), 2)
				cv2.rectangle (input_frame, (int(myObjectX - 1), int(myObjectY - 1)), (int(myObjectX + 1), int(myObjectY + 1)), (0, 0, 255), 2)
		else:
			if rect is not None:
				[top, left, bottom, right] = rect
				cv2.rectangle (input_frame, (int(left), int(top)), (int(right), int(bottom)), (0, 0, 255), 2)
				cv2.rectangle (input_frame, (int(myObjectX - 1), int(myObjectY - 1)), (int(myObjectX + 1), int(myObjectY + 1)), (0, 0, 255), 2)
		
		#(h, w) = input_frame.shape[:2]
		input_frame = cv2.resize(input_frame, (640, 480))
		# ==================================================================================
		        
		#cv2.putText (input_frame, "From video_detection_node", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
		#	1, (0, 0, 255), 3)
			
		# convert to Image and send			
		self.imagePublisher_.publish(self.br.cv2_to_imgmsg(input_frame))
	
def main(args=None): 
    # Initialize the ROS 2 library for Python
    rclpy.init(args=args)
  
    # Create the node
    uavDetection_publisher = UAVdetection()
  
    # Spin the node so the callback function is called.
    rclpy.spin(uavDetection_publisher)
  
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    uavDetection_publisher.destroy_node()
  
    # Shutdown the ROS 2 client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
