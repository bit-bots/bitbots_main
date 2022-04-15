import cv2

import vision_modules.yoeo_handler as yoeo

config = {'caching': True, 'yoeo_nms_threshold': .5, 'yoeo_conf_threshold': 0.5}
model_path = "/home/bitbot/bitbots/bitbots_meta/bitbots_vision/bitbots_vision/models/2021_12_06_flo_torso21_yoeo_7"

my_handler = yoeo.YOEOHandlerOpenVino(config, model_path)

test_image = "/home/bitbot/bitbots/YOEO/data/samples/montreal-game02_aa_000001.png"

img = cv2.imread(test_image, -1)
print(img.shape)

my_handler._compute_new_prediction(img, .5, .5)

field = my_handler.get_segmentation_mask_for("background")
cv2.imshow("background", field * 255)
field = my_handler.get_segmentation_mask_for("field")
cv2.imshow("field", field * 255)
field = my_handler.get_segmentation_mask_for("lines")
cv2.imshow("lines", field * 255)
cv2.waitKey()