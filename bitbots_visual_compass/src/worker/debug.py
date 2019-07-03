import cv2


class Debug:
    def __init__(self):
        pass

    def print_debug_info(self, image, state, callback):
        debug_image = image.copy()

        font = cv2.FONT_HERSHEY_SIMPLEX
        bottom_left_corner_of_text = (10, 35)
        font_scale = 1
        font_color = (255, 255, 255)
        line_type = 2

        cv2.putText(debug_image, "SIDE {} | Confidence {}".format(*state),
                    bottom_left_corner_of_text,
                    font,
                    font_scale,
                    font_color,
                    line_type)

        callback(debug_image)
