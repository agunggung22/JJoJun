import cv2
import numpy as np

zero = np.zeros([300, 300], np.uint8)
zero[0:155, :] = 255
cv2.namedWindow("zeros", cv2.WINDOW_NORMAL)
cv2.imshow("zeros", zero)
cv2.waitKey(0)
cv2.destroyAllWindows()
