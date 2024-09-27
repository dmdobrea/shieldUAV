#!/usr/bin/env python
# -*- coding: utf-8 -*-

import 	rclpy
from 	rclpy.node 	import Node
import 	numpy 		as np
from 	rclpy.clock import Clock
from 	rclpy.qos 	import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg 		import OffboardControlMode
from px4_msgs.msg 		import TrajectorySetpoint
from px4_msgs.msg 		import VehicleStatus
from px4_msgs.msg 		import VehicleAttitude
from px4_msgs.msg 		import VehicleCommand
from geometry_msgs.msg 	import Twist, Vector3


class VelocityOffboardControl(Node):

	def __init__(self):
		super().__init__('velocity_control_node')
        
		qos_profile = QoSProfile(
			reliability	= QoSReliabilityPolicy.BEST_EFFORT,
			durability	= QoSDurabilityPolicy.TRANSIENT_LOCAL,
			history		= QoSHistoryPolicy.KEEP_LAST,
			depth=1
		)

		#================================== Create subscriptions from PX4 autopilot
		# 
		self.status_sub   = self.create_subscription( VehicleStatus,  '/fmu/out/vehicle_status',   	self.vehicle_status_callback, qos_profile)
		self.attitude_sub = self.create_subscription( VehicleAttitude,'/fmu/out/vehicle_attitude', 	self.attitude_callback,       qos_profile)
        
		# from central command node <= FROM HERE THIS ENTIRE NODE IS COMMANDED
		self.offboard_velocity_sub = self.create_subscription( Twist, '/offboard_velocity_cmd', 	self.offboard_velocity_callback, qos_profile)
        
        #================================== Create publishers
		# required for commands like arm, takeoff
		self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand,      '/fmu/in/vehicle_command',                     10)
        
		# required for working on offboard mode: velocity, position, acceleration, etc.
		self.publisher_offboard_mode    = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode',               qos_profile)
        
		# set effectivly the movement through: velocity, position, acceleration, etc. values
		self.publisher_trajectory       = self.create_publisher(TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',                 qos_profile)
        
        #================================== Create timers
		# creates callback function for the command loop period, this just should be more than 2Hz. 
		# Commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
		timer_period = 0.02  # seconds
		self.timer = self.create_timer(timer_period, self.cmdloop_callback)

		# variables
		self.velocity 	 = Vector3()

		self.nav_state   = VehicleStatus.NAVIGATION_STATE_MAX
		self.arm_state   = VehicleStatus.ARMING_STATE_ARMED
		self.failsafe    = False
		self.flightCheck = False        

		self.yaw     	 = 0.0  #yaw value we send as command
		self.trueYaw 	 = 0.0  #current yaw value of drone
        
		self.i			 = 0
        
	def state_arming(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
		self.get_logger().info("Arm command send")

	def state_takeoff(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0) # param7 is altitude in meters
		self.get_logger().info("Takeoff command send")

	def state_offboard(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)

	# Arms the vehicle
	def arm(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
		self.get_logger().info("Arm command send")

	# Takes off the vehicle to a user specified altitude (meters)
	def take_off(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0) # param7 is altitude in meters
		self.get_logger().info("Takeoff command send")

	#publishes command to /fmu/in/vehicle_command
	def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
		msg = VehicleCommand()
		msg.param1 = param1
		msg.param2 = param2
		msg.param7 = param7    # altitude value in takeoff command
		msg.command = command  # command ID
		msg.target_system = 1  # system which should execute the command
		msg.target_component = 1  # component which should execute the command, 0 for all components
		msg.source_system = 1  # system sending the command
		msg.source_component = 1  # component sending the command
		msg.from_external = True
		msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
		self.vehicle_command_publisher_.publish(msg)

	#receives and sets vehicle status values 
	def vehicle_status_callback(self, msg):

		if (msg.nav_state != self.nav_state):
			self.get_logger().info(f"NAV_STATUS: {self.decodeNavState(msg.nav_state)}")
        
		if (msg.arming_state != self.arm_state):
			self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

		if (msg.failsafe != self.failsafe):
			self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
		if (msg.pre_flight_checks_pass != self.flightCheck):
			self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

		self.nav_state   = msg.nav_state
		self.arm_state   = msg.arming_state
		self.failsafe    = msg.failsafe
		self.flightCheck = msg.pre_flight_checks_pass

	def decodeNavState (self, navState):
		if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
			return "AUTO TAKEOFF"
		if (navState == VehicleStatus.NAVIGATION_STATE_POSCTL):
			return "Position control mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
			return "AUTO LOITER"
		if (navState == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
			return "OFFBOARD"
		if (navState == VehicleStatus.NAVIGATION_STATE_MANUAL):
			return "manual mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_ALTCTL):
			return "altitude control mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION):
			return "auto mission mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_RTL):
			return "auto return to launch mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_ACRO):
			return "acro mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_DESCEND):
			return "descend mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_TERMINATION):
			return "termination mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_STAB):
			return "stabilized mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_LAND):
			return "land"
		if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_FOLLOW_TARGET):
			return "auto follow"
		if (navState == VehicleStatus.NAVIGATION_STATE_ORBIT):
			return "orbit in a circle"
		else:
			return navState

	#receives Twist commands from Teleop and converts NED -> FLU
	def offboard_velocity_callback(self, msg):
		#implements conversion from NED to FLU frame
 
		# X (FLU) is -Y (NED)
		self.velocity.x = -msg.linear.y

		# Y (FLU) is X (NED)
		self.velocity.y = msg.linear.x

		# Z (FLU) is -Z (NED)
		self.velocity.z = -msg.linear.z

		# A conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.trueYaw)
		self.yaw = msg.angular.z

	#receives current trajectory values from drone and grabs the yaw value of the orientation
	def attitude_callback(self, msg):
		orientation_q = msg.q

		#trueYaw is the drones current yaw value
		self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
			1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
        
	#publishes offboard control modes and velocity as trajectory setpoints
	def cmdloop_callback(self):
		
		if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD: 
			# Publish offboard control modes
			offboard_msg = OffboardControlMode()
            
			offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            
			offboard_msg.position     = False
			offboard_msg.velocity 	  = True
			offboard_msg.acceleration = False
			offboard_msg.attitude     = False
			offboard_body_rate        = False
			
			self.publisher_offboard_mode.publish(offboard_msg)            

			# Compute velocity in the world frame
			cos_yaw = np.cos(self.trueYaw)
			sin_yaw = np.sin(self.trueYaw)
			velocity_world_x = (self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
			velocity_world_y = (self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)

			# Create and publish TrajectorySetpoint message with NaN values for position and acceleration
			trajectory_msg = TrajectorySetpoint()
            
			trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            
			trajectory_msg.velocity[0] 		= velocity_world_x
			trajectory_msg.velocity[1] 		= velocity_world_y
			trajectory_msg.velocity[2] 		= self.velocity.z
			trajectory_msg.position[0] 		= float('nan')
			trajectory_msg.position[1] 		= float('nan')
			trajectory_msg.position[2] 		= float('nan')
			trajectory_msg.acceleration[0] 	= float('nan')
			trajectory_msg.acceleration[1] 	= float('nan')
			trajectory_msg.acceleration[2] 	= float('nan')
			trajectory_msg.yaw 				= self.yaw
			trajectory_msg.yawspeed 		= float('nan')

			self.publisher_trajectory.publish(trajectory_msg)
		else:
			self.i = self.i + 1
			if self.i % 20 == 0:
				self.get_logger().info("Not in offboard mode")


def main(args=None):
	
	# Initialize the ROS 2 library for Python
    rclpy.init(args=args)
    
	# Create the node
    offboard_velocity_control = VelocityOffboardControl()

	# Spin the node so the callback function is called.
    rclpy.spin(offboard_velocity_control)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    offboard_velocity_control.destroy_node()
    
    # Shutdown the ROS 2 client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
