# An autonomous shieldUAV to protect and save lives
This repository was developed in the context of the **Pervasive AI Developer Contest with AMD** (link: https://www.hackster.io/contests/amd2023), and it is our contribution (Dobrea Matei-Ștefan & Dobrea Dan-Marius) to the contest.

You can read the entire project report we made from the following link: https://www.hackster.io/520087/an-autonomous-shielduav-to-protect-and-save-lives-99a722. Within this report, a series of links are made that explain each component separately, so if you want to understand their role, it is recommended to go through the report.

## Image acquisition application
An important project component is the application with which we acquired the images. A hardware component connected to **PMOD** port 1 (connector J2) of the **Kria KR260** development board is required for this application to work. You have a picture of it below.

## Detection algorithm
One of the main contributions of our team to this contest is the development of a **new algorithm** that combines **YOLO** and **correlation tracking algorithm** to follow all the time-designated **UAV** in all the frames. Even if the **YOLO** is a state-of-the-art detection algorithm, its performance is not 100%. Moreover, due to the quantization of the model (in order to be able to run on **Kria KR260**), the performance of the **YOLO** model decreases further.

A short video demonstration is presented in the following video:

[![Everything Is AWESOME](https://github.com/dmdobrea/shieldUAV/blob/main/Images/Youtube_3UAV.PNG)](https://youtu.be/67_oauHjYTA "Everything Is AWESOME")

Starting with the 58th second of the video, as the **UAV** progresses, the original **YOLOv3** model (_**YOLOv3 DarkNet**_) can follow the **UAV** without any problem. But the **YOLOv3 model** quantized and compiled for **Kria KR260** (YOLOv3*) cannot follow the **UAV** up to the moment with the time mark 1:06 – this is the result of quantization that generates some accuracy losses of the **YOLO** model. But, based on the **correlation tracking algorithm** (**CTA**), the combined model (**YOLOv3* + CTA**) can perfectly follow the drone on 21 more frames.
Please see the rest of the video in which the combined algorithm (**YOLOv3*** in blue + **CTA** in red) can track the UAV without missing any frames.

The code used to get the result from the previous video is the one from here: _**07_YOLOv3-tyny_UAV_plane**_.





