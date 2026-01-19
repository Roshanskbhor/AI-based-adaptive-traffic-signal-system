# AI-based-adaptive-traffic-signal-system


Overview

This project implements an AI-based adaptive traffic signal control system that dynamically adjusts green signal duration based on real-time vehicle density. The system integrates ESP32, ESP32-CAM, YOLO-based vehicle detection, FastAPI, and MQTT to enable intelligent, lane-wise traffic management.

Unlike traditional fixed-time traffic lights, this system responds to actual traffic conditions, helping to reduce congestion and improve traffic flow efficiency.\


System Architecture
![image alt](https://github.com/Roshanskbhor/AI-based-adaptive-traffic-signal-system/blob/5544415bded202fefc64322aaada7c81ba1b7f68/System_architecture.jpeg)

The system consists of three major components:

1]ESP32 Traffic Controller
Controls traffic signals, servo motor, and decision logic.

2]ESP32-CAM Module
Captures lane images when triggered by the controller.

3]Backend Server (FastAPI + YOLO)
Processes images to detect and count vehicles and communicates results using MQTT.



Working Principle

1.The ESP32 controller manages the current traffic lane and green signal timing.
2.Before switching lanes, the controller triggers the ESP32-CAM module.
3.The ESP32-CAM captures an image of the selected lane.
4.The captured image is sent to a FastAPI server over Wi-Fi.
5.A YOLO-based object detection model detects and counts vehicles in the image.
6.The detected vehicle count is published to the ESP32 controller via MQTT.
7.The ESP32 dynamically calculates the green signal duration for the next lane.
8.A servo motor rotates the camera toward the next lane 5 seconds before lane switching

Hardware Components
![image alt](https://github.com/Roshanskbhor/AI-based-adaptive-traffic-signal-system/blob/8ef6aed291357184681d5f8fce77b3d31c396bfd/Prototype_picture.jpeg)
ESP32 Dev Module (Traffic Controller)
ESP32-CAM (AI Thinker Module)
Servo Motor (Camera rotation)
Traffic Signal LEDs (Red, Yellow, Green)
External 5V Power Supply (for servo motor)


Communication Protocols

Wi-Fi – Image transmission from ESP32-CAM to backend
MQTT – Vehicle count communication from backend to ESP32 controller


Key Features

1.Real-time adaptive traffic signal timing
2.Lane-wise vehicle detection using computer vision
3.MQTT-based low-latency communication
4.Servo-based camera positioning
5.Scalable and cost-effective system design
