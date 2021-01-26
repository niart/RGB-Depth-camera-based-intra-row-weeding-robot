
import cv2
import cv2 as cv

src = cv2.imread("real.png", cv2.IMREAD_COLOR)

hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)

h, s, v = cv2.split(hsv)

cv.namedWindow("hsv", cv.WINDOW_NORMAL)

cv2.resizeWindow("hsv", 800, 600)

cv2.imshow("hsv", hsv)

cv.namedWindow("h", cv.WINDOW_NORMAL)

cv2.resizeWindow("h", 800, 600) 
cv2.imshow("h", h)
status_h = cv2.imwrite('/home/bonjour/Desktop/picker/h_replace.png',h)
 
print("h replaced!",status_h)

cv.namedWindow("s", cv.WINDOW_NORMAL)

cv2.resizeWindow("s", 800, 600)

cv2.imshow("s", s)

cv.namedWindow("v", cv.WINDOW_NORMAL)

cv2.resizeWindow("v", 800, 600)

cv2.imshow("v", v)
status_v = cv2.imwrite('/home/bonjour/Desktop/picker/v_replace.png', v)
 
print("v replaced!",status_v)

cv2.waitKey(0)

cv2.destroyAllWindows()