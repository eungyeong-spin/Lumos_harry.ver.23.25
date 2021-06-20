#!/usr/bin/env python

import jetson.utils
import rospy, cv2, numpy
from std_msgs.msg import Int64

rospy.init_node('follower2')
pub = rospy.Publisher('cx', Int64, queue_size = 1)

camera = jetson.utils.videoSource(argv=["csi://0", "--input-flip=horizontal"])

while True:
    img = camera.Capture()
    image = jetson.utils.cudaToNumpy(img)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    lower = numpy.array([22, 93, 0], dtype="uint8")
    upper = numpy.array([45, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(image, image, mask=mask)
    #gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
    #blur = cv2.GaussianBlur(gray, (5, 5), 0)
    #ret, thresh1 = cv2.threshold(blur, 150, 255, cv2.THRESH_BINARY_INV)
    
    #contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #contours = contours[0] if len(contours) == 2 else contours[1]
    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        cv2.line(result, (cx, 0), (cx, 720), (255, 0, 0), 2)
        cv2.line(result, (0, cy), (1280, cy), (255, 0, 0), 2)
        #cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 2)
        
        pub.publish(cx)
        
    cv2.imshow('frame', result)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
