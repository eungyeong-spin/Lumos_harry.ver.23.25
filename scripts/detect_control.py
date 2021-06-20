#!/usr/bin/env python

import rospy
import time
import jetson.inference
import jetson.utils

import argparse
import sys
import os
from std_msgs.msg import String

# parse the command line
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

# load the object detection network
net = jetson.inference.detectNet(argv=['--model=models/ssd-mobilenet.onnx', '--labels=models/labels.txt', '--input-blob=input_0', '--output-cvg=scores', '--output-bbox=boxes'])
rospy.init_node('detect_control')
pub = rospy.Publisher('/jetbot_motors/cmd_str', String, queue_size=3)
pub2 = rospy.Publisher('/alarm', String, queue_size=1)

# create video sources & outputs
camera = jetson.utils.videoSource(argv=["csi://0", "--input-flip=horizontal"])
display = jetson.utils.videoOutput("display://0")

# process frames until the user exits
while display.IsStreaming():
	# capture the next image
	img = camera.Capture()
	detections = net.Detect(img, overlay=opt.overlay)
	num = len(detections)
	if num > 0:
		for detection in detections:
 			class_name = net.GetClassDesc(detection.ClassID)
			if class_name == "STOP":
				pub.publish("stop")
				rospy.loginfo("Stop sign is detected")
			elif class_name == "LEFT":
				pub.publish("right")
				rospy.loginfo("Left sign is detected")
			elif class_name == "RIGHT":
				pub.publish("left")
				rospy.loginfo("Right sign is detected")
			elif class_name == "SLOW" or class_name == "CHILD":
				pub.publish("slow")
				rospy.loginfo("school zone")
			elif class_name == "POTHOLE":
				pub2.publish("pothole")
			elif class_name == "CRACK":
				pub2.publish("crack")
                
                
	else:
		mot = "forward"
		pub.publish("backward")
		rospy.loginfo("Nothing is detected")

	display.Render(img)
	display.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))
	net.PrintProfilerTimes()

	if not camera.IsStreaming() or not display.IsStreaming():
		break
