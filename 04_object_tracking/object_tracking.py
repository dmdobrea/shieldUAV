from tools.object_center import ObjCenter
from tools.pid           import PID
from multiprocessing     import Manager
from multiprocessing     import Process
from imutils.video       import VideoStream

import threading
import argparse
import signal
import time
import sys
import cv2
import mctrl_comm
import time


sys.path.append("/home/ubuntu/kamikazeUAV/teste/object_tracking/tools")
import pid

from threading import Lock

# define the range for the motors
servoRange = (-45, 45)
thread_frame1 = int(0)
thread_frame2 = int(0)
writing_frame = 1
want_frame = False

# function to handle keyboard interrupt
def signal_handler(sig, frame):
	# disable the servos and gracefully exit
	print("[INFO] You pressed `ctrl + c`! Exiting...")
	#### pth.servo_enable(1, False)
	#### pth.servo_enable(2, False)
	sys.exit()
	
def frame_eater(cap, camera_access):
	global thread_frame1
	global thread_frame2
	global writing_frame
	global want_frame
	
	while 1:

		if writing_frame == 1:
			_, thread_frame1 = cap.read()
			camera_access.acquire(1)
			if not want_frame:
				writing_frame = 2
			camera_access.release()
		else:
			_, thread_frame2 = cap.read()
			camera_access.acquire(1)
			if not want_frame:
				writing_frame = 1
			camera_access.release()

	
def obj_center(objX, objY, centerX, centerY, frame_counter):
	global want_frame
	global writing_frame
	global thread_frame1
	global thread_frame2
	
	# set the signal trap to handle keyboard interrupt and then
	signal.signal(signal.SIGINT, signal_handler)
	
	# initialize the object center finder
	obj = ObjCenter("tf_yolov3_voc.xmodel")
	
	# start the video stream and wait for the camera to warm up
	#inputSource = "rtsp://admin:Mateidan@192.168.1.199:554" #-- netcam
	inputSource = 0
	cap = cv2.VideoCapture(inputSource)
	time.sleep(2.0)
	
	camera_access = Lock()
	
	threadFrameEater = threading.Thread(target=frame_eater, args=tuple([cap, camera_access]))
	threadFrameEater.start()

	while(type(thread_frame1) == int or type(thread_frame2) == int):
		pass
	# loop indefinitely
	#cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)
	while True:
		# grab the frame from the threaded video stream and flip it
		start = time.time()
		
		
		camera_access.acquire(1)
		want_frame = True
		camera_access.release()
		
		if writing_frame == 1:
			frame = thread_frame2
		else:
			frame = thread_frame1
			
		camera_access.acquire(1)
		want_frame = False
		camera_access.release()
		
		
		#_, frame = cap.read()
		
		frame = cv2.resize(frame, (640, 480))
        
        # vertically (since our camera was upside down)
		#frame = cv2.flip(frame, 0)
		
		# calculate the center of the frame as this is (ideally) where
		# we will we wish to keep the object
		(H, W) = frame.shape[:2]
		centerX.value = W // 2
		centerY.value = H // 2	
		
		((objX.value, objY.value), rect, alg) = obj.update(frame, (centerX.value, centerY.value), 4)
		
		if alg == 1:
			if rect is not None:
				[top, left, bottom, right] = rect
				cv2.rectangle (frame, (int(left), int(top)), (int(right), int(bottom)), (255, 0, 0), 2)
				cv2.rectangle (frame, (int(objX.value - 1), int(objY.value - 1)), (int(objX.value + 1), int(objY.value + 1)), (0, 0, 255), 2)
		else:
			if rect is not None:
				[top, left, bottom, right] = rect
				cv2.rectangle (frame, (int(left), int(top)), (int(right), int(bottom)), (0, 0, 255), 2)
				cv2.rectangle (frame, (int(objX.value - 1), int(objY.value - 1)), (int(objX.value + 1), int(objY.value + 1)), (255, 0, 0), 2)
			
		
		# display the frame to the screen
		cv2.imshow("Pan-Tilt Face Tracking", frame)
		cv2.waitKey(1)

		if(frame_counter.value == 100000):
			frame_counter.value = 0
		else:
			frame_counter.value += 1

		
def pid_process(output, p, i, d, objCoord, centerCoord, modifier, frame_counter):
	# set the signal trap to handle keyboard interrupt, then create a
	# PID and initialize it
	signal.signal(signal.SIGINT, signal_handler)
	
	pid_obj = pid.PID(kP = p.value, kI = i.value, kD = d.value)
	pid_obj.initialize()
	
	current_frame = frame_counter.value

	# loop indefinitely
	while True:
		error = centerCoord.value - objCoord.value
		if output.value > 1 and error * modifier > 0:
			out_of_bounds = True
		elif output.value < -1 and error * modifier < 0:
			out_of_bounds = True
		else:
			out_of_bounds = False
		
		adjustment = pid_obj.update(error, 0, out_of_bounds)
		output.value = output.value + adjustment * modifier
		print(f"{output.value:.2f}, {centerCoord.value - objCoord.value:.2f}         ", end='\r')

		while(current_frame == frame_counter.value):
			time.sleep(0.01)

		current_frame = frame_counter.value
		
def in_range(val, start, end):
	# determine the input vale is in the supplied range
	return (val >= start and val <= end)

def set_servos(x_pos, y_pos, frame_counter):
	# set the signal trap to handle keyboard interrupt
	signal.signal(signal.SIGINT, signal_handler)
	
	arm = mctrl_comm.io_expansion_board(False, PWM2_min = -0.8, PWM2 = -0.6)

	current_frame = frame_counter.value

	# loop indefinitely
	while True:
		#print(f"{x_pos.value:.2f}", end='\r')
		arm.PWM_value_set(PWM1 = x_pos.value)
		arm.PWM_value_set(PWM2 = y_pos.value)
		
		arm.update_state()

		while(current_frame == frame_counter.value):
			time.sleep(0.01)

		current_frame = frame_counter.value
			
			
# check to see if this is the main body of execution
if __name__ == "__main__":
	
	# start a manager for managing process-safe variables
	with Manager() as manager:
		# enable the servos
		#### pth.servo_enable(1, True)
		#### pth.servo_enable(2, True)
		
		# set integer values for the object center (x, y)-coordinates
		centerX = manager.Value("i", 0)
		centerY = manager.Value("i", 0)

		# set integer values for the object's (x, y)-coordinates
		objX = manager.Value("i", 0)
		objY = manager.Value("i", 0)

		# pan and tilt values will be managed by independent PIDs
		x_pos = manager.Value("i", 0)
		y_pos = manager.Value("i", -0.3)

		# set PID values for panning
		xP = 0.3
		xI = 0.013
		xD = 0.038
		xP = manager.Value("f", xP / 1000)
		xI = manager.Value("f", xI / 1000)
		xD = manager.Value("f", xD / 1000)

		# set PID values for tilting
		yP = 0.3
		yI = 0.017
		yD = 0.042
		yP = manager.Value("f", yP / 1000)
		yI = manager.Value("f", yI / 1000)
		yD = manager.Value("f", yD / 1000)
		#tiltP = manager.Value("f", 0.11)
		#tiltI = manager.Value("f", 0.10)
		#tiltD = manager.Value("f", 0.002)
		
		# we have 4 independent processes
		# 1. objectCenter - finds/localizes the object
		# 2. panning   - PID control loop determines panning angle
		# 3. tilting   - PID control loop determines tilting angle
		# 4. setServos - drives the servos to proper angles based
		#                on PID feedback to keep object in center

		frame_counter = manager.Value("i", 0)

		processObjectCenter = Process(target=obj_center,  args=(objX, objY, centerX, centerY, frame_counter))
		processPanning      = Process(target=pid_process, args=(x_pos, xP, xI, xD, objX, centerX, -1, frame_counter))
		processTilting      = Process(target=pid_process, args=(y_pos, yP, yI, yD, objY, centerY, 1, frame_counter))
		processSetServos    = Process(target=set_servos,  args=(x_pos, y_pos, frame_counter))

		# start all 4 processes
		processObjectCenter.start()
		processPanning.start()
		processTilting.start()
		processSetServos.start()

		# join all 4 processes
		processObjectCenter.join()
		processPanning.join()
		processTilting.join()
		processSetServos.join()

		# disable the servos
		#### pth.servo_enable(1, False)
		#### pth.servo_enable(2, False)
