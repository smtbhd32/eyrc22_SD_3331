# E-Yantra 2022: Drone Automation with ROS2 and OpenCV

This project was part of the E-Yantra 2022 competition, where I built an autonomous drone that surveys a city and looks for yellow objects. The drone transmits the geolocation of detected objects via a ROS topic. A QGIS script then reads the ROS topic and displays the detected yellow blocks on a city map in real-time.

## Watch Demo

You can watch the demonstration on YouTube by clicking the video below:

<p align="center">
  <a href="https://youtu.be/TrLvPJrHu5E" target="_blank">
    <img src="https://github.com/smtbhd32/eyrc22_SD_3331/raw/main/video%20thumbnail.png" alt="Watch the video" width="80%" height="auto" />
  </a>
</p>


## Project Overview

The drone receives images of the city from an installed camera. These images are processed to find yellow objects of specific dimensions. If a yellow object is detected, the drone’s image is compared with a city map (in .tif format with geocoordinates). Feature extraction is performed on both the drone's image and the city map using the SIFT (Scale-Invariant Feature Transform) algorithm. A comparison matrix is then created using RANSAC (Random Sample Consensus) to derive the geolocation of the yellow object.

The project was divided into two stages:

- **Stage 1**: Developed a ROS controller script with PID for stable flight in a Gazebo simulation. Integrated object detection using OpenCV, transmitted geolocation data of detected yellow blocks, and visualized this data in real-time using QGIS.
- **Stage 2**: Assembled and automated a nano drone with the GEPRC411 flight controller and Banana Pi. Encountered several challenges during the drone automation process.

## Stage 1: ROS2 Controller & Object Detection

- **ROS2 Controller**: Developed a PID-based controller to ensure stable flight in a Gazebo simulation.
- **Object Detection**: Implemented autonomous object detection using OpenCV (Python), specifically targeting yellow blocks.
- **Geolocation**: The geolocation of the detected yellow blocks is transmitted and plotted in real-time on QGIS for dynamic visualization of their locations on a city map.
- **Algorithms**: Utilized OpenAI's SIFT and RANSAC algorithms for robust object recognition and geolocation calculation based on the comparison between the drone's image and the city map.

## Stage 2: Drone Assembly & Automation

- **Drone Assembly**: Assembled a nano drone with the GEPRC411 flight controller and Banana Pi. Faced challenges in the process of automating the drone.
- **Autonomous Navigation**: Integrated object detection for autonomous navigation, enabling the drone to locate and interact with detected yellow objects based on their geolocation.


## Acknowledgments

- **IIT Bombay** for providing the robotic kit and guidance.
- **FH Aachen - University of Applied Sciences** and **Alberto Ezquerro Baraibar** for inspiration.
- **OpenAI** for contributing to the development of advanced algorithms like SIFT and RANSAC.

---
