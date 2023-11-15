import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class TurtleBotController:
    def __init__(self):
        # Initialize the ROS node with the name 'turtlebot_controller'
        rospy.init_node('turtlebot_controller', anonymous=True)
        
        # Create a publisher to send velocity commands on the '/cmd_vel' topic
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Create a subscriber to receive depth images from the '/camera/depth/image_raw' topic
        self.image_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.callback)
        
        # Initialize the CvBridge for image conversion
        self.bridge = CvBridge()

    def move_robot(self, linear_vel, angular_vel):
        # Create a Twist message to set linear and angular velocities
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        
        # Publish the Twist message to control the robot's movement
        self.cmd_vel_pub.publish(twist_msg)

    def depth_image_processing(self, depth_image):
        # Set a depth threshold to detect nearby obstacles
        obstacle_threshold = 1.0

        # Apply thresholding to the depth image
        _, thresholded_image = cv2.threshold(depth_image, obstacle_threshold, 255, cv2.THRESH_BINARY)

        # Find contours in the thresholded image
        contours, _ = cv2.findContours(thresholded_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Take actions based on the depth image information
        if contours:
            # If contours (detected objects) are present, stop the robot to avoid collisions
            linear_vel = 0.0
            angular_vel = 0.0
            self.move_robot(linear_vel, angular_vel)
        else:
            # If no obstacles are detected, move forward
            linear_vel = 0.2
            angular_vel = 0.0
            self.move_robot(linear_vel, angular_vel)

    def callback(self, msg):
        try:
            # Convert the ROS image to OpenCV format using CvBridge
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            # Process the depth image
            self.depth_image_processing(depth_image)
            
            # Display the depth image (for debugging purposes)
            cv2.imshow("Depth Image", depth_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    try:
        # Initialize the TurtleBotController class and start the ROS event loop
        turtlebot_controller = TurtleBotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
