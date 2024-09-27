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

# Import all the required libraries
import rclpy                                # Python Client Library for ROS 2
from   rclpy.qos           import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from   rcl_interfaces.msg  import ParameterDescriptor
from   rclpy.node          import Node      # Handles the creation of nodes
from   sensor_msgs.msg     import Image     # Imports the built-in Image message type
import cv2                                  # OpenCV library - import before cv_bridge
from   cv_bridge           import CvBridge  # Package to convert between ROS and OpenCV Images

import imagezmq
import socket

# 0 to 100, higher is better quality, 95 is OpenCV default
jpeg_quality = 65

class VideoDataSubscriber(Node):
    """
    Here I create an VideoDataSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a sugegestive name
        super().__init__('video_broadcast_node')

        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,                  # RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL,               # RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history     = QoSHistoryPolicy.KEEP_LAST,                        # RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth       = 1 )
                
        # creates a parameter(s) with the name and default value(s)
        verbose_descriptor = ParameterDescriptor(description='Add verbosity to the running node.')
        self.declare_parameter('verbose', 1, verbose_descriptor)

        serverIP_descriptor = ParameterDescriptor(description='Set the video server IP.')
        self.declare_parameter('server_ip', '192.168.100.117', serverIP_descriptor)
        
        sourceSel_descriptor = ParameterDescriptor(description='Source selection: 0 - from videp publisher node, 1 - from image detection/classification node')
        self.declare_parameter('source_sel', 1, sourceSel_descriptor)        
        
        #get parameters        
        verbose    = self.get_parameter('verbose').get_parameter_value().integer_value
        server_ip  = self.get_parameter('server_ip').get_parameter_value().string_value
        source_sel = self.get_parameter('source_sel').get_parameter_value().integer_value
        
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic or video_detection
        if   source_sel == 0:
            self.subscription = self.create_subscription( Image, 'video_frames', self.video_listener_callback, qos_profile)
        elif source_sel == 1:    
            self.subscription = self.create_subscription( Image, 'video_detection', self.video_listener_callback, qos_profile)
        else:
            if verbose == 1: print ("[ERR._VB] : The selected subscription does not exist!")
            sys.exit()

        self.subscription # prevent unused variable warning
      
        # initialize the ImageSender object with the socket address of the server
        self.sender = imagezmq.ImageSender(connect_to="tcp://{}:5555".format(server_ip))
        if verbose == 1: print ("[INFO_VB] : Frames will be send to = {}".format(server_ip))

        # get the host name
        self.kr260_Name = socket.gethostname()
        if verbose == 1: print ("[INFO_VB] : Host name = {}".format(self.kr260_Name))
        print (" ")

        # Used to convert images between ROS 2 and OpenCV
        self.br = CvBridge()
   
    def video_listener_callback(self, newFrame):
        """
        Callback function - when a new frame has arrived, a jump is made here
        """
        verbose   = self.get_parameter('verbose').get_parameter_value().integer_value
    
        # Display the message on the console
        if verbose == 1:
          self.get_logger().info('VB: Received a new video frame')
 
        # Convert ROS Image message to OpenCV image
        send_frame = self.br.imgmsg_to_cv2(newFrame)

        # get the encoded image as array of bytes
        ret_code, jpg_buffer = cv2.imencode(".jpg", send_frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])

        # send the frame to the server
        self.sender.send_jpg(self.kr260_Name, jpg_buffer) 

  
def main(args=None):

    # Initialize the ROS 2 library for Python
    rclpy.init(args=args)
  
    # Create the node
    video_subscriber = VideoDataSubscriber()
  
    # Spin the node so the callback function is called.
    rclpy.spin(video_subscriber)
  
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
  
    # Shutdown the ROS 2 client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
