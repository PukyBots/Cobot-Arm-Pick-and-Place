import cv2
import numpy as np
cap = cv2.VideoCapture(1)


while True:
    ret, frame = cap.read()
   
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
   
    mask = cv2.inRange(hsv, lower_red, upper_red)
    result = cv2.bitwise_and(frame, frame, mask=mask)
   
    cv2.imshow("Original Cam", frame)
    cv2.imshow("Mask", result)  
   
    if cv2.waitKey(1) == ord('x'):
        break
cap.release()
cv2.destroyAllWindows()
