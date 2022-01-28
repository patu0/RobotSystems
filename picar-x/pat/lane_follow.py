import cv2
frame = cv2.imread('C:/Users/97pat/Desktop/tape.png')
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
cv2.imshow("hsv", hsv)
cv2.waitKey(0)