#!/usr/bin/env python

import rospy
import time
import jetson.inference
import jetson.utils
import cv2, numpy

import argparse
import sys
import os
from std_msgs.msg import String, Int64

parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.detectNet.Usage() +
                                 jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use") 

is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

try:
	opt = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

net = jetson.inference.detectNet(argv=['--model=models/ssd-mobilenet.onnx', '--labels=models/labels.txt', '--input-blob=input_0', '--output-cvg=scores', '--output-bbox=boxes'])
rospy.init_node('detect_control')

pub = rospy.Publisher('/jetbot_motors/cmd_str', String, queue_size=3)
pub2 = rospy.Publisher('cx', Int64, queue_size = 1)
pub3 = rospy.Publisher('/alarm', String, queue_size=1)


camera = jetson.utils.videoSource(argv=["csi://0", "--input-flip=horizontal"])
display = jetson.utils.videoOutput("display://0")
global cx
global cy

def line_following(image):
    image = jetson.utils.cudaToNumpy(img)
    image = image[400:720, 0:1280]
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    
    lower = numpy.array([22, 93, 0], dtype="uint8")
    upper = numpy.array([45, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(image, image, mask=mask)
    global cx
    global cy
    
    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        cv2.line(result, (cx, 0), (cx, 720), (255, 0, 0), 2)
        cv2.line(result, (0, cy), (1280, cy), (255, 0, 0), 2)
        
    pub2.publish(cx)
    cv2.imshow('frame', result)
    
while True:
	img = camera.Capture()
	detections = net.Detect(img, overlay=opt.overlay)
    
	num = len(detections)
	if num > 0:
		for detection in detections:
 			class_name = net.GetClassDesc(detection.ClassID)
			class_conf = 100*detection.Confidence
			class_box = int(detection.Area)
			if class_name == "STOP" and class_box >= 72900:
				pub.publish("stop")
				rospy.loginfo("Stop sign is detected")
			elif class_name == "LEFT" and class_box >= 72900:
				pub.publish("left")
				rospy.loginfo("Left sign is detected")
			elif class_name == "RIGHT"and class_box >= 72900:
				pub.publish("right")
				rospy.loginfo("Right sign is detected")
			elif class_name == "SLOW" or class_name == "CHILD" and class_box >= 50000 and class_conf >= 70:
				pub.publish("slow")
				rospy.loginfo("school zone")
				rospy.sleep(4)
			elif class_name == "POTHOLE" and class_conf >= 80:
				pub3.publish("pothole")
				rospy.loginfo("Pothole is detected")
				line_following(img)
			elif class_name == "CRACK" and class_conf >= 80:
				pub3.publish("crack")
				rospy.loginfo("Crack is detected")
                		line_following(img)
			else:
				rospy.loginfo("Confidence: %d" , int(class_conf))
				line_following(img)
	else:
		line_following(img)
		rospy.loginfo("Nothing is detected")

	display.Render(img)
	display.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))
	net.PrintProfilerTimes()

	if not camera.IsStreaming() or not display.IsStreaming():
		break
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
