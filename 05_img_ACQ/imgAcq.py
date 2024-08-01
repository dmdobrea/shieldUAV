import os
import time
import datetime
import cv2
import threading
import multiprocessing
import sys

from threading import Lock

sys.path.append('/usr/local/share/pynq-venv/lib/python3.10/site-packages')
os.environ['PYNQ_JUPYTER_NOTEBOOKS'] = '/root/jupyter_notebooks'
os.environ['BOARD'] = 'KV260'
os.environ['XILINX_XRT'] = '/usr'
os.environ['PATH'] = os.environ.get('PATH') + ':/usr/local/share/pynq-venv/bin/microblazeel-xilinx-elf/bin/'
os.environ['LD_LIBRARY_PATH'] = '/usr/lib'

from kv260 import BaseOverlay
from pynq.lib import MicroblazeLibrary
#--------------
inputSource = "rtsp://admin:Mateidan@192.168.1.199:554" #-- netcam
#inputSource = 0
#"/home/mdobrea/matei/Python\\ Code/recs"
outputFolder = "/home/ubuntu/Desktop/video-out"
FourCC_code = 'XVID'
videoSegmentLength = 25 #in seconds
continuous = False
#--------------

filming_master_ok = False
filming_master_thread = 0
camFilming = False
blink_type = 2
button1_value = 1
button2_value = 1

def date_name_maker(x):
    def func(x):
        if x < 10:
            return "0" + str(x)
        return str(x)

    return func(x.year) + "-" + func(x.month) + "-" + func(x.day) + "_" + func(x.hour) + "-" + func(x.minute) + "-" + func(x.second)


def frame_eater(inputSource, end, camera_access, cap, video_error):
    while end.value:
        camera_access.acquire(1)
        ret, _ = cap.read()
        camera_access.release()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            video_error.value = True
            break


def filming_process(inputSource, end, outputFile, videoNumber, video_error, FourCC_code = 'XVID'):
    #"/home/mdobrea/Desktop/test.h264"
    camera_access = Lock()
    current_videoNumber = videoNumber.value

    print("input: {}".format(inputSource))
    #print(outputFile.format(videoNumber.value))

    cap = cv2.VideoCapture(inputSource)
    if not cap.isOpened():
        print("Cannot open camera")
        video_error.value = True
        exit()
    
    frame_eater_thread = threading.Thread(target=frame_eater, args=tuple([inputSource, end, camera_access, cap, video_error]))
    frame_eater_thread.start()

    fourcc = cv2.VideoWriter_fourcc(*FourCC_code)
    out = cv2.VideoWriter(outputFile.format(videoNumber.value), fourcc, 25.0, (640, 480))

    while end.value:
        if current_videoNumber != videoNumber.value:
            out.release()
            out = cv2.VideoWriter(outputFile.format(videoNumber.value), fourcc, 25.0, (640, 480))
            current_videoNumber = videoNumber.value
        # Capture frame-by-frame
        camera_access.acquire(1)
        ret, frame = cap.read()
        camera_access.release()

        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            video_error.value = True
            break

        frame = cv2.resize(frame, (640, 480))

        #cv2.imshow("Preview", frame)
        #if cv2.waitKey(1) == ord('q'):
        #    print("can't stop it like this :))")
        
        out.write(frame)
        # Display the resulting frame
        #cv2.imshow('frame', frame)
    
    frame_eater_thread.join()
    camera_access.acquire(1)
    cap.release()
    camera_access.release()
    out.release()
    cv2.destroyAllWindows()


def filming_master(inputSourceGiven):
    global filming_master_ok
    global outputFolder
    global videoSegmentLength
    global continuous
    global FourCC_code
    global video_error

    start = time.time()
    end = multiprocessing.Value('b', True)

    x = datetime.datetime.now()
    folderNumber = date_name_maker(x)

    if inputSourceGiven == 0:
        os.mkdir(outputFolder + "/recording_USB_" + folderNumber)
        temp_output_file = outputFolder + "/recording_USB_" + folderNumber + "/vid_USB_{}.avi"
    else:
        os.mkdir(outputFolder + "/recording_ETH_" + folderNumber)
        temp_output_file = outputFolder + "/recording_ETH_" + folderNumber + "/vid_ETH_{}.avi"

    videoNumber = multiprocessing.Value('i', 0)
    film_process = multiprocessing.Process(target=filming_process, args=(inputSourceGiven, end, temp_output_file, videoNumber, video_error, FourCC_code))
    print("starting film {}".format(folderNumber))
    film_process.start()
    if continuous:
        while filming_master_ok:
            pass
    else:
        while filming_master_ok:
            if time.time() - start >= videoSegmentLength:
                videoNumber.value += 1
                start = time.time()
    end.value = False
    film_process.join()
    print("done film {}".format(folderNumber))


def button_pressed():
    global filming_master_thread
    global filming_master_ok
    global inputSource
    global LED1
    print("normal")

    if filming_master_ok:
        filming_master_ok = False
        filming_master_thread.join()
    else:
        filming_master_thread = threading.Thread(target=filming_master, args=(inputSource, ))
        filming_master_ok = True
        filming_master_thread.start()



class LED_BLINK_TYPE:
    ETH = 0
    USB = 1
    NOTHING = 2
    ERROR = 3


def blink_light(LED, LED_type, pmod_mutex):
    global blink_type
    while 1:
        while blink_type != LED_type:
            pmod_mutex.acquire(1)
            lib.gpio_write(LED, 1)
            pmod_mutex.release()
            for i in range(5):
                time.sleep(0.1)
                if blink_type == LED_type or blink_type == LED_BLINK_TYPE.ERROR:
                    break
            if blink_type == LED_type or blink_type == LED_BLINK_TYPE.ERROR:
                break

            pmod_mutex.acquire(1)
            lib.gpio_write(LED, 0)
            pmod_mutex.release()
            for i in range(5):
                time.sleep(0.1)
                if blink_type == LED_type or blink_type == LED_BLINK_TYPE.ERROR:
                    break
            if blink_type == LED_type or blink_type == LED_BLINK_TYPE.ERROR:
                break

        if blink_type == LED_type or blink_type == LED_BLINK_TYPE.ERROR:
            pmod_mutex.acquire(1)
            lib.gpio_write(LED, 1)
            pmod_mutex.release()
        while blink_type == LED_type or blink_type == LED_BLINK_TYPE.ERROR:
           pass 

def button_reading_continuously(pmod_mutex):
    global button1_value
    global button2_value
    global button1
    global button2
    global video_error
    global blink_type

    while 1:
        pmod_mutex.acquire(1)
        button1_value = button1.read()
        button2_value = button2.read()
        pmod_mutex.release()
        #print(f"{button1_value}, {button2_value}")
        if video_error.value:
            blink_type = LED_BLINK_TYPE.ERROR
        time.sleep(0.1)

if __name__ == "__main__":
    
    #LOOK INTO THIS
    #temp = cv2.VideoCapture(0)
    #print(temp.get(cv2.CAP_PROP_FPS))
    #^^^^^^^^^^^^^^^^^^^^^^^^^^

    # init for the FPGA
    base = BaseOverlay('base.bit')
    lib = MicroblazeLibrary(base.PMOD0, ['gpio'])

    video_error = multiprocessing.Value('b', False)
    pmod_mutex = Lock()

    # maps the pins
    LED1 = lib.gpio_open(3)
    LED2 = lib.gpio_open(7)
    button1 = lib.gpio_open(0)
    button2 = lib.gpio_open(4)

    lib.gpio_set_direction(LED1, lib.GPIO_OUT)
    lib.gpio_set_direction(LED2, lib.GPIO_OUT)
    lib.gpio_set_direction(button1, lib.GPIO_IN)
    lib.gpio_set_direction(button2, lib.GPIO_IN)


    blink_thread1 = threading.Thread(target=blink_light, args=tuple([LED1, LED_BLINK_TYPE.ETH, pmod_mutex]))
    blink_thread1.start()
    blink_thread2 = threading.Thread(target=blink_light, args=tuple([LED2, LED_BLINK_TYPE.USB, pmod_mutex]))
    blink_thread2.start()

    button_reading_thread = threading.Thread(target=button_reading_continuously, args=tuple([pmod_mutex, ]))
    button_reading_thread.start()

    while 1:
        pmod_mutex.acquire(1)
        button11_value = button1_value
        button22_value = button2_value
        pmod_mutex.release()		
		
        while button11_value != 0 and button22_value != 0:
            pmod_mutex.acquire(1)
            button11_value = button1_value
            button22_value = button2_value
            pmod_mutex.release()
            #print(f"{button11_value}, {button22_value}")
        #print(f"{button11_value}, {button22_value}")
        

        if camFilming:
            video_error.value = False
            blink_type = LED_BLINK_TYPE.NOTHING
            button_pressed()
            camFilming = False
            time.sleep(1)
        else:
            camFilming = True
            if button11_value == 0:
                print("ETH")
                inputSource = "rtsp://admin:Mateidan@192.168.1.199:554"
                blink_type = LED_BLINK_TYPE.ETH
                button_pressed()
                time.sleep(1)
            elif button22_value == 0:
                print("USB")
                inputSource = 0
                blink_type = LED_BLINK_TYPE.USB
                button_pressed()
                time.sleep(1)
 
