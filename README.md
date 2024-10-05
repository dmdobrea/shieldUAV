# An autonomous shieldUAV to protect and save lives
This repository was developed in the context of the **Pervasive AI Developer Contest with AMD** (link: https://www.hackster.io/contests/amd2023), and it is our contribution (Dobrea Matei-Ștefan & Dobrea Dan-Marius) to the contest.

You can read the entire project report we made from the following link: https://www.hackster.io/520087/an-autonomous-shielduav-to-protect-and-save-lives-99a722. Within this report, a series of links are made that explain each component separately, so if you want to understand their role, it is recommended to go through the report.

## Image acquisition application
An important project component is the application with which we acquired the images. A hardware component connected to **PMOD** port 1 (connector J2) of the **Kria KR260** development board is required for this application to work. You have a picture of it below.

<img src="https://github.com/dmdobrea/shieldUAV/blob/main/Images/PMOD_LED_butt.PNG" width="480"/>

The schematic of this hardware component is as follows:

<img src="https://github.com/dmdobrea/shieldUAV/blob/main/Images/PMOD.png" width="640"/>

## Detection algorithm
One of the main contributions of our team to this contest is the development of a **new algorithm** that combines **YOLO** and **correlation tracking algorithm** to follow all the time-designated **UAV** in all the frames. Even if the **YOLO** is a state-of-the-art detection algorithm, its performance is not 100%. Moreover, due to the quantization of the model (in order to be able to run on **Kria KR260**), the performance of the **YOLO** model decreases further.

A short video demonstration is presented in the following video:

<div align="left">
      <a href="https://youtu.be/67_oauHjYTA">
         <img src="https://github.com/dmdobrea/shieldUAV/blob/main/Images/Youtube_3UAV.PNG" style="width:60%;">
      </a>
</div>

Starting with the 58th second of the video, as the **UAV** progresses, the original **YOLOv3** model (_**YOLOv3 DarkNet**_) can follow the **UAV** without any problem. But the **YOLOv3 model** quantized and compiled for **Kria KR260** (YOLOv3*) cannot follow the **UAV** up to the moment with the time mark 1:06 – this is the result of quantization that generates some accuracy losses of the **YOLO** model. But, based on the **correlation tracking algorithm** (**CTA**), the combined model (**YOLOv3* + CTA**) can perfectly follow the drone on 21 more frames.
Please see the rest of the video in which the combined algorithm (**YOLOv3*** in blue + **CTA** in red) can track the UAV without missing any frames.

The code used to get the result from the previous video is the one from here: _**07_YOLOv3-tyny_UAV_plane**_.

## Neuronal models

An essential component of the activities carried out within the **Pervasive AI Developer Contest with AMD** was obtaining neural models capable of detecting two types of **UAV** systems: drones and airplanes. As we explained in the project report, we could only get the neural models for **YOLOv3 tiny**, **YOLOv3**, **YOLOv7 tiny**, and **YOLOv7**. The performances obtained after running them on the **DPU** module inside the **Kria KR260** development system are those presented in the table below.

<img src="https://github.com/dmdobrea/shieldUAV/blob/main/Images/YOLO%20performance.PNG" width="640"/>
From the above table, it can be seen that the performances of the neural models in **YOLOv3 tiny** and **YOLOv7 tiny** are somewhat similar (24.9 ms versus 30.5 ms) with an advantage in favor of the **YOLOv3 tiny** model. Unfortunately, decoding the outputs of the neural model (performed within the evaluated function) takes a long time in the **YOLOv7 tiny** model due to the large number of boxes detected. This led us to use the **YOLOv3 tiny** neural model.

All these neural models can be downloaded from the following link:

https://drive.google.com/drive/folders/11QGuT2wW5nQgVnQSLV9aVMeFdSbEkN2d

from where you can download the YOLO_UAV_Neuronal_Models.zip archive with all these neural models.

## The software components developed in ROS2

The software part of the shieldUAV drone was developed entirely in Python. 
	For its development we used the support of Robot Operating System 2 (ROS). The application consists of five ROS 2 nodes that are embedded into a single ROS package.
	Each node is associated with a specific Python file (***video.py***, ***detection.py***, ***broadcast.py***, ***control.py*** and ***velocity.py*** - see 11_kamikaze_uav), being mainly a specific process that is performing a specific task.
 A frame obtained from **video_publisher_node** is detected inside the video_detection_node and the result in this configuration is streamed via **video_broadcast_node**. 
	The shieldUAV package embeds a specific Python module that allows the user to configure the source of the video input stream. In this mode, the input images stream can be taken from: (a) an input video file, (b) a **USB** camera, or (c) a network camera.
	The **video_broadcast_node** is also able to connect to **video_publisher_node** in order to get raw images. The streaming is done based on ZeroMQ streaming protocol to the base station based on the existing WiFi link. 
	To have a real-time solution, the images obtained from a specific topic are compressed to the jpeg format and streamed.
	The **velocity_control_node** node connects to the PX4 autopilot and it implements the control of the shieldUAV system based on the velocity mode. Also, it interrogates the drone status, allowing us to know the status of the drone in every single moment. This node can arm, disarm, takeoff, land etc., the drone according with necessities resulting from the offboard control.
	The connection between the **velocity_control_node** and the **PX4** is based on a bridge component. This bridge component is executed on the companion computer like a service when the companion computer starts its executions.
	The **main_control_node** uses the information from **video_detection_node** and manage the **velocity_control_node** node in order to achieve its goal: to identifies an aggressor drones, to follow and  finally crashes into them.

<img src="https://github.com/dmdobrea/shieldUAV/blob/main/Images/rqt.png" width="1200"/>

	
