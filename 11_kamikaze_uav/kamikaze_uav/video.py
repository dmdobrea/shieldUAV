#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import the necessary libraries
import rclpy 				                                # Python Client Library for ROS 2
from   rclpy.qos                    import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from   rcl_interfaces.msg           import ParameterDescriptor  # In order to manage parameters
from   rclpy.node                   import Node 	            # Handles the creation of nodes
from   sensor_msgs.msg              import Image 	            # Imports the built-in Image message type
import cv2 				                                    	# OpenCV library - import before cv_bridge
from   cv_bridge                    import CvBridge 	        # Package to convert between ROS and OpenCV Images
from   .submodules.VideoDevices     import getVideoStream

class VideoImagePublisher(Node):
    """
    Here I create an VideoImagePublisher class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('video_publisher_node')
    
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,                  # RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL,               # RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history     = QoSHistoryPolicy.KEEP_LAST,                        # RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth       = 1 )

        # creates a parameter(s) with the name and default value(s)
        verbose_descriptor = ParameterDescriptor(description='Add verbosity to the running node.')
        self.declare_parameter('verbose', 1, verbose_descriptor)
      
        videoSource_descriptor = ParameterDescriptor(description='0 => USB cam    1 => Ethernet cam.')
        self.declare_parameter('videoSource', 0, videoSource_descriptor)

        width_descriptor = ParameterDescriptor(description='Image width [640]')
        self.declare_parameter('width', 640, width_descriptor)

        height_descriptor = ParameterDescriptor(description='Image height [480]')
        self.declare_parameter('height', 480, height_descriptor)
        
        # path to the video, like:
        # 		'/home/user/FilmTest.mp4' or 
		#		'None'
		#
		# in my case '/home/ubuntu/DeTest.mp4'
        filePath_descriptor = ParameterDescriptor(description='File path to the mp4 video')
        self.declare_parameter('file_path', '/home/ubuntu/DeTest.mp4', filePath_descriptor)  
        
        # Create the publisher. This publisher will publish images of type Image
        # to the "video_frames" topic.
        self.imagePublisher_ = self.create_publisher(Image, 'video_frames', qos_profile)
      
        # We will publish a message every 0.1 seconds
        timer_period = 0.2  # seconds
      
        # Create the timer
        self.timer = self.create_timer(timer_period, self.videoTimer_callback)
         
        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        my_verbose  = self.get_parameter('verbose').get_parameter_value().integer_value
        my_width    = self.get_parameter('width').get_parameter_value().integer_value
        my_height   = self.get_parameter('height').get_parameter_value().integer_value
        my_videoS   = self.get_parameter('videoSource').get_parameter_value().integer_value
        my_filePath = self.get_parameter('file_path').get_parameter_value().string_value

        self.vs = getVideoStream (my_filePath, 0, my_videoS, my_width, my_height, my_verbose)

        # Used to convert image format between ROS 2 and OpenCV
        self.br = CvBridge()
   
    def videoTimer_callback(self):
        """
        Timer Callback function.
        """
        # gets the parameter from the node
        my_verbose = self.get_parameter('verbose').get_parameter_value().integer_value
        
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.vs.read()
          
        if ret == True:
            # Image exist & nou publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.imagePublisher_.publish(self.br.cv2_to_imgmsg(frame))

            # Display the message on the console
            if my_verbose == 1:
                self.get_logger().info('Video frame published')
        else:
            # Display the message on the console
            if my_verbose == 1:
                self.get_logger().info('No video frame to publish')
  
def main(args=None): 
    # Initialize the ROS 2 library for Python
    rclpy.init(args=args)
  
    # Create the node
    videoImage_publisher = VideoImagePublisher()
  
    # Spin the node so the callback function is called.
    rclpy.spin(videoImage_publisher)
  
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    videoImage_publisher.destroy_node()
  
    # Shutdown the ROS 2 client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()


