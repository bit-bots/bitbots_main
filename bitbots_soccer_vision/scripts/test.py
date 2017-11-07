import cv2
import sys
import vision

vision = vision.Vision()
for imagename in sys.argv[1:]:
    print('reading image ', imagename)
    image = cv2.imread(imagename)
    vision.handle_image(image=image)
