Blue Color Detector
This code is a ROS (Robot Operating System) node in Python
used for processing images obtained from a camera and publishing a control command to a joint in a robotic system.

Requirements
Python 3
OpenCV
ROS
Numpy

Packages
MYROBOT_control package contains PID contol parameters and launch files for gazebo_control package
per_project package contains blue detector node, necesarry urdf files and mesh files to simulate robot in Gazebo
vision_opencv contains image_geometry and cv_bridge packages that are utilized by pet_project package and blue_detector node


Usage
Run the code with the following command.

$ rosrun blue_detector blue_detector.py

How it works
The node creates an instance of CvBridge class, which is used to convert between ROS Image messages and OpenCV images. 
The node then subscribes to the /camera/image_raw topic, which is used to receive raw images from the camera.
The received images are processed in the callback method, 
which converts the received image from a ROS Image message to a OpenCV format using the CvBridge instance and passes it to the detect_blue method for processing. 
The method also includes error handling to print any exceptions that may occur during the conversion.
The detect_blue method detects blue color in an image using OpenCV. If blue color is detected, it calls the manipluate_joint method with a value of 0.0. 
If blue color is not detected, the function calls the manipluate_joint method with a value of -0.3. 
The manipluate_joint method manipulates a prismatic joint (linear actuator) by publishing a std_msgs/float64 message to the /joint1_position_controller/command topic.
Finally, the node creates a publisher to the /joint1_position_controller/command topic, which is used to send control commands to the joint. 
If the visualize flag is set, the result image is displayed using cv2.imshow().

Limitations
The code is only tested with a specific robotic system and may not work with other systems without modifications. 
The range of blue color in HSV is defined as lower_blue and upper_blue and may need to be adjusted based on the specific camera and lighting conditions.