#!/usr/bin/env python

"""follower_ros.py: Robot will follow the Yellow Line in a track"""

__author__  = "Arjun S Kumar"

import jetson.utils
import rospy, cv2, numpy
from geometry_msgs.msg import Twist
from lumos.msg import control
import argparse
import sys

class Follower:
  def __init__(self):
    self.cmd_vel_pub = rospy.Publisher('/jetbot_motors/cmd_vel',
                                       Twist, queue_size=1)
    self.twist = Twist()

  def follow(self, img):
    image = jetson.utils.cudaToNumpy(img)
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    # change below lines to map the color you wanted robot to follow
    lower_yellow = numpy.array([ 10,  10,  10])
    upper_yellow = numpy.array([255, 255, 250])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # CONTROL starts
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # CONTROL ends

    cv2.imshow("window", mask)
    cv2.waitKey(3)


rospy.init_node('follower')
follower = Follower()

camera = jetson.utils.videoSource(argv=["csi://0", "--input-flip=horizontal"])
display = jetson.utils.videoOutput("display://0")

while True:
    img = camera.Capture()
    follower.follow(img)

