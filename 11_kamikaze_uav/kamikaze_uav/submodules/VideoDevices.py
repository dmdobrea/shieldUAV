import os
import sys
import cv2

def getVideoStream (inputFile, camNo, isIt_Eth, frameWidth, frameHeight, verb):
    if inputFile != 'None':
        #=======================================================================
        # Based on the input grab a reference to the video file
        if os.path.isfile(inputFile):
            if verb == 1:
                print("[INFO_VS] : Opening input video file: {}".format(inputFile))
            vs = cv2.VideoCapture(inputFile)
        else:
            if verb == 1:
                print("[ERR._VS] : The file {} does not exist! The program will be terminated.".format(inputFile))
            sys.exit()
    else:
        if verb == 1:
            print("[INFO_VS] : Opening input from CAM...")
        if isIt_Eth == 0:
            # it is a USB cam
            videoPath = "/dev/video{}".format(int(camNo)*2)
            if not os.path.exists(videoPath):           #os.path.isfile - check it is a regular file
                if verb == 1:
                    print( "[ERR._VS] : The camera /dev/video{} is not connected to system! The program will be terminated.".format(int(camNo)*2) )
                sys.exit()
            
            vs = cv2.VideoCapture(int(camNo)*2)
            if not vs.isOpened():
                if verb == 1:
                    print( "[ERR._VS] : I can't open the camera /dev/video{} connected to system! The program will be terminated.".format(int(camNo)*2) )
                sys.exit()
                
            vs.set(cv2.CAP_PROP_FRAME_WIDTH,  frameWidth)
            vs.set(cv2.CAP_PROP_FRAME_HEIGHT, frameHeight)
            if frameWidth != vs.get(cv2.CAP_PROP_FRAME_WIDTH) or frameHeight != vs.get(cv2.CAP_PROP_FRAME_HEIGHT):
                if verb == 1:
                    print( "[ERR._VS] : Unable to set your resolution. Frame resolution set to: {} x {}".format(cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT) )
                
        elif isIt_Eth == 1:
            # it is an Ethernet cam
                
            vs = cv2.VideoCapture("rtsp://admin:Mateidan@192.168.1.199:554/H264/ch1/sub/cv_stream")
            if not vs.isOpened():
                if verb == 1:
                    print( "[ERR._VS] : I can't open network camera! The program will be terminated." ) 
                sys.exit()
        else:
            if verb == 1:
                print( "[ERR._VS] : You select an unknow cam board! The program will be terminated." )
            sys.exit()
    return vs


