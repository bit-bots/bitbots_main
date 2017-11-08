from vision_modules import ball, classifier, live_classifier, horizon, color, debug_image
import cv2


class Vision:

    def __init__(self):
        self.field_color_detector = color.ColorDetector('../config/fieldColor.yaml')  # Todo: set right path
        self.cascade = cv2.CascadeClassifier('../classifier/cascadeNew.xml')  # Todo: set path
        self.ball_classifier = live_classifier.LiveClassifier('../models/classifier_01')  # Todo: set path
        self.debug = True

    def handle_image(self, image):
        horizon_detector = horizon.HorizonDetector(image, self.field_color_detector)
        ball_finder = ball.BallFinder(image, self.cascade)
        # Todo: filter balls under horizon
        ball_classifier = classifier.Classifier(image, self.ball_classifier, ball_finder.get_candidates())
        found_ball = None
        if ball_classifier.get_top_candidate()[1] > 0.5:  # Todo: set real threshold
            found_ball = ball_classifier.get_top_candidate()
        print(found_ball)
