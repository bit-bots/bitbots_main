from .vision_modules import color, classifier, live_classifier, ball, horizon
import cv2

class Vision:

    def __init__(self):
        self.field_color_detector = color.ColorDetector('../../config/fieldColor.yaml')  # Todo: set right path
        self.cascade = cv2.CascadeClassifier('')  # Todo: set path
        self.ball_classifier = live_classifier.LiveClassifier('')  # Todo: set path

    def handle_image(self, image):
        horizon_detector = horizon.HorizonDetector(image, self.field_color_detector)
        ball_finder = ball.BallFinder(image, self.cascade)
        # Todo: filter balls under horizon
        ball_classifier = classifier.Classifier(image, self.ball_classifier, ball_finder.get_candidates())
        found_ball = None
        if ball_classifier.get_top_candidate()[1] > 0.5:  # Todo: set real threshold
            found_ball = ball_classifier.get_top_candidate()

