#import matplotlib.pyplot as plt
import cv2
import cv2 as cv
import numpy as np
#import numpy as np
image = np.zeros((300, 300, 255), np.uint8)
cv2.imshow("img", image)
cv2.waitKey(0) #keep showing