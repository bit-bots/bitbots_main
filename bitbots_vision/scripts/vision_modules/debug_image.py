import cv2


class DebugImage:

    def __init__(self, image):
        self.raw_image = image[:]

    def draw_horizon(self, horizon_points, color):
        """
        draws a line where the horizon algorithm found the horizon
        :param horizon_points list of coordinates of the horizon:
        :return void:
        """
        for i in range(len(horizon_points) - 1):
            cv2.line(self.raw_image,
                     horizon_points[i],
                     horizon_points[i+1], color)

    def draw_ball_candidates(self, ball_candidates, color):
        """
        draws a circle around every coordinate where a ball candidate was found
        :param ball_candidates list of cooordinates of ball candidates:
        :return void:
        """
        for candidate in ball_candidates:
            cv2.circle(self.raw_image,
                       (candidate[0] + candidate[2]//2,
                        candidate[1] + candidate[3] // 2),
                       candidate[3] // 2, color)

    def draw_points(self, points, color, rad=2):
        for point in points:
            cv2.circle(self.raw_image, point, rad, color)

    def imshow(self):
        """
        saves the image in current directory as "img.png" with edits made to the image from the other functions
        :return void:
        """
        # cv2.imwrite('img.png', self.raw_image)
        cv2.imshow('Debug Image', self.raw_image)
        cv2.waitKey(0)
