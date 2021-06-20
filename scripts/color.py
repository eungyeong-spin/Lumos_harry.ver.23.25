import jetson.utils
import numpy as np
import cv2

def find_yellow(image):
    original = image.copy()
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.array([22, 93, 0], dtype="uint8")
    upper = np.array([45, 255, 255], dtype="uint8")
    mask = cv2.inRange(image, lower, upper)
    
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    for c in cnts:
        x,y,w,h = cv2.boundingRect(c)
        cv2.rectangle(original, (x, y), (x + w, y + h), (36,255,12), 2)
    
    cv2.imshow('mask', mask)
    cv2.imshow('original', original)

camera = jetson.utils.videoSource(argv=["csi://0", "--input-flip=horizontal"])

while True:
    img = camera.Capture()
    image = jetson.utils.cudaToNumpy(img)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    find_yellow(image)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break