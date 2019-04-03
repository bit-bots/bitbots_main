import numpy as np
import cv2


img1 = cv2.imread('frame1.png', 0)
img2 = cv2.imread('frame4.png', 0)

img1 = np.float32(img1)
img2 = np.float32(img2)

ret = cv2.phaseCorrelate(img1, img2)

def show(image):
    cv2.imshow('image',image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


#show(img)

print(ret)