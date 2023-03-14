#!/usr/bin/env python3
import rospy, cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

class ImageConverter:
    def __init__(self):
        """
            This code defines the __init__ method for a ROS (Robot Operating System) node in Python. 
            The node is used for processing images obtained from a camera and publishing a control command to a joint in a robotic system.
            The code creates an instance of CvBridge class, which is used to convert between ROS Image messages and OpenCV images.
            
            The node then subscribes to the "/camera/image_raw" topic, which is used to receive raw images from the camera. 
            The image_sub variable is a rospy.Subscriber object that subscribes to the topic and specifies a callback function, 
            self.callback, to be called every time a new image is received.

            Finally, the code creates a publisher to the "/joint1_position_controller/command" topic, which is used to send control commands to the joint. 
            The pub variable is a rospy.Publisher object that publishes messages of type Float64 to the topic, with a queue size of 10. 
            This means that up to 10 messages can be stored in the queue if the joint is unable to process the messages as fast as they are being published.
        """
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.pub = rospy.Publisher('/joint1_position_controller/command', Float64, queue_size=10)

        self.visualize = False # visualize the blue color coming from image topic

    def callback(self, data):
        """
            The callback method is used to process images received from the "/camera/image_raw" topic in a ROS node. 
            The method converts the received image from a ROS Image message to a OpenCV format using the CvBridge instance and passes it to the detect_blue method for processing. 
            The method also includes error handling to print any exceptions that may occur during the conversion.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.detect_blue(cv_image)
        except CvBridgeError as e:
            print(e)

    def manipluate_joint(self, position):
        """
            manipulate prismatic joint (linear actuator)
            Publiches the std_msgs/float64 message
        """
        msg = Float64()
        msg.data = position
        self.pub.publish(msg)

    def detect_blue(self, image):
        """
        This function detects blue color in an image using OpenCV.
        The input image is first converted from BGR to HSV color space. The range of blue color in HSV is defined as lower_blue and upper_blue.

        A mask is created by thresholding the HSV image to get only blue colors. The original image is bitwise-ANDed with the mask to get the result with only blue colors.
        The function then checks if blue color is detected by counting the non-zero elements in the mask. 

        If blue color is detected, the function calls the manipulate_joint() method with a value of 0.0. If blue color is not detected, 
        the function calls the manipulate_joint() method with a value of -0.3.

        Finally, if the visualize flag is set, the result image is displayed using cv2.imshow().
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the range of blue color in HSV
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask=mask)

        # Check if blue color is detected
        if cv2.countNonZero(mask) > 0:
            print("Blue color detected!")
            self.manipluate_joint(0.0)
        else:
            self.manipluate_joint(-0.3)


        if self.visualize:
            cv2.imshow("Image", res)
            cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("blue_detector", anonymous=True)
    rospy.loginfo("blue detector is starting ...")
    ic = ImageConverter()
    rospy.spin()