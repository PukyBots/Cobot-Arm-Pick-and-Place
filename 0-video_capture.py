import cv2
cap1 = cv2.VideoCapture(1)
while True:
    ret, frame = cap1.read()
    cv2.imshow('Webcam', frame)
    if cv2.waitKey(1) == ord('x'):
        break  
cap1.release()
cv2.destroyAllWindows()
