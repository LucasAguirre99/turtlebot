import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import sys, tty, termios
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import cv2.cv as cv
import math

#Lecture de clavier sans Enter
def getChar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

#Conversion de radian en degree et inversement
def convertToRad(degree):
    return degree * math.pi * 1.0 / 180

def convertToDegree(rad):
    return rad * 180.0 / math.pi

"""def callback(msg):
    bridge = CvBridge()
    # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
        # The depth image is a single-channel float32 image
        depth_image = bridge.imgmsg_to_cv(msg, "32FC1")
    except CvBridgeError & e:
        print(e)
"""

#Pour déplacer le robot
def move_robot(linear_vel, angular_vel):
    twist_msg = Twist()
    twist_msg.linear.x = linear_vel
    twist_msg.angular.z = angular_vel
    # Publica el mensaje en el tópico de velocidad del robot
    pub.publish(twist_msg)

#traitement approfondi des images
def callback(msg):
    bridge = CvBridge()
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # Effectuer des opérations de traitement d'image ici
        processed_image = depth_image_processing(depth_image)
        # Affiche l'image traitée (à des fins de débogage uniquement)
        cv2.imshow("Processed Image", processed_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print(e)

#Contrôle de robot basé sur l'image de profondeur
def depth_image_processing(depth_image):
    # Logique de traitement de l'image de profondeur et de génération de commandes de contrôle
    # (par exemple, éviter les obstacles si la profondeur est inférieure à un seuil)
    linear_vel = 0.2
    angular_vel = 0.0  # Il ne tourne pas dans cet exemple
    move_robot(linear_vel, angular_vel)

