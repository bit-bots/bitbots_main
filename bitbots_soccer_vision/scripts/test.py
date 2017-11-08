import cv2
import sys
import vision

vision = vision.Vision()
for imagename in sys.argv[1:]:
    print('reading image ', imagename)
    image = cv2.imread(imagename)
    a = cv2.getTickCount()
    vision.handle_image(image=image)
    b = cv2.getTickCount()
    print( (b - a) / cv2.getTickFrequency())
