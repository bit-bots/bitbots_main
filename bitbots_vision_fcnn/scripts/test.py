import cv2
import sys
import vision

vision = vision.Vision()
for imagename in sys.argv[2:]:
    print('reading image ', imagename)
    image = cv2.imread(imagename)
    a = cv2.getTickCount()
    for x in range(int(sys.argv[1])):
        vision.handle_image(image=image)
    b = cv2.getTickCount()
    print((b - a) / float(cv2.getTickFrequency()))
