# python3 tracking.py --video file --path vid1-4.avi

from tools.object_center import ObjCenter

import os
import cv2
import sys
import time
import argparse

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", required=True, help="from where video information will be get: file, cam or not")
ap.add_argument("-p", "--path", help="path to the input video")
ap.add_argument("-o", "--out",  help="video output yes or no")
args = vars(ap.parse_args())

if args["video"] == "file":
	if os.path.isfile(args["path"]):
		captureStream = cv2.VideoCapture(args["path"])
	else:
		print("[INFO] : The file {} does not exist! The program will be terminated.".format(args["path"]))
		sys.exit()
	
if args["video"] == "cam":
	captureStream = cv2.VideoCapture(0)


obj = ObjCenter("KR260_yolov3-tiny_UAV.xmodel")

if args["video"] == "file" or args["video"] == "cam":
	print ("\n[INFO] : start detection process")
	counter_frame = 0
	
	time1 = time.time()
	
	fourcc = cv2.VideoWriter_fourcc(*'XVID')
	out = cv2.VideoWriter("out-vid.avi", fourcc, 25.0, (640, 480))
	
	while 1:
		# Read an image from the input stream
		(success, input_image) = captureStream.read()	# the image is in BGR format!
		
		# if we are viewing a video and we did not grab a frame, 
		# then we have reached the end of the video
		if args["video"] == "file" and not success:
			print ("\n[INFO] : I have reached the end of the video")	
			break
		
		(H, W) = input_image.shape[:2]
		centerX = W // 2
		centerY = H // 2
			
		myObjectDet = obj.update(input_image, (centerX, centerY), 0)
		
		((myObjectX, myObjectY), rect, alg) = myObjectDet
		
		if alg == 1:
			if rect is not None:
				[top, left, bottom, right] = rect
				cv2.rectangle (input_image, (int(left), int(top)), (int(right), int(bottom)), (255, 0, 0), 2)
				cv2.rectangle (input_image, (int(myObjectX - 1), int(myObjectY - 1)), (int(myObjectX + 1), int(myObjectY + 1)), (0, 0, 255), 2)
		else:
			if rect is not None:
				[top, left, bottom, right] = rect
				cv2.rectangle (input_image, (int(left), int(top)), (int(right), int(bottom)), (0, 0, 255), 2)
				cv2.rectangle (input_image, (int(myObjectX - 1), int(myObjectY - 1)), (int(myObjectX + 1), int(myObjectY + 1)), (0, 0, 255), 2)
		
		cv2.imshow ("UAV detection", input_image)	
		cv2.waitKey(1)
		
		out.write(input_image)

		counter_frame = counter_frame + 1

	time2 = time.time()
	
	fps = counter_frame/(time2-time1)
	print("\n[INFO] : Performance => {} FPS".format(fps))
	
del obj
print("[INFO] : program detection ended!")
