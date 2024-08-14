# SEARCH, DETECTION, AND LOCALIZATION OF AN OBJECT USING AERIAL DRONE
This is the final project for the course ENAE788M: Hands on Autonomous Aerial Robotics. The project focuses on state estimation, object detection using Yolo v5, and localization of an object using a tracking camera

Manuscript: https://shorturl.at/3LLEL
Demo: https://youtu.be/0m49_NMmelI

### Software dependencies
```
ROS Foxy
PX4 1.14 or later
```

### Running the code
1. *Clone the package to your ros workspace*
2. *Clone the following package in the ros workspace for px4_msgs dependency*
```
git clone https://github.com/PX4/px4_msgs.git -b release/1.14
```
3. *Build and source the workspace*
```
cd <path_to_ros_ws>
colcon buiild
source install/setup.bash
```
4. *Run the trained Yolov5 obeject detecetion model provided by ModalAI*
```
voxl-tflite-server
```
5. *Run the following node to get the detected object details on a ros topic '/tflite_data'*
```
ros2 run voxl_mpa_to_ros2 voxl_mpa_to_ros2_node
```
6. *To get the position coordinates of apriltag in the drone inertial frame*
```
ros2 run apriltag_localize_node
```
7. *To localize the detected object in the drone inertial frame run the following commands in two seperate terminals*
```
ros2 run localization_package inertial_camera_coord.py
ros2 run localization_package localization_inertial_node
```
8. *Arm the drone and takeoff. Run the following node and switch to 'offboard' mode to start the object search*
```
ros2 run px4_ros_com search_for_tag
```
9. *Subscribe to the '/object/pose_inertial' to get the coordinates of the detected object in the camera frame*
```
ros2 topic echo /object/pose_inertial
```
*Messsages are published on the topic whenever the drone detectcs a backpack/handbag in the camera*

### References
Vision-based Target Localization from a Fixed-wing Unmanned Aerial Vehicle:
https://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=2544&context=facpub \
Deep Learning with VOXL-TFLite-Server: https://docs.modalai.com/voxl-tflite-server-0_9/


