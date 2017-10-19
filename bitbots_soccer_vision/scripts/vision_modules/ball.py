import cv2
import numpy as np


class BallFinder:

    def __init__(self, classifier, cascade):
        self._candidates = None
        self._candidates_classified = False  # remembers if balls got classified
        self._ball = None
        self._classifier = classifier
        self._cascade = cascade

        self._classify_threshold = 0.5
        self._debug = False

    def get_candidates(self, image):
        if self._candidates is None:
            self._candidates_classified = False
            pass  # Todo: Find candidates
        return self._candidates

    def get_ball(self, image, horizon):
        if not self._candidates_classified:
            self._ball = self._classify_candidates(image)
        return self._ball

    def _classify_candidates(self, image):
        ball_candidates = self.get_candidates(image)
        self._candidates_classified = True  # remember that balls got classified
        if ball_candidates:
            batch = list()
            for item in ball_candidates:
                radius = int(item.diameter / 2)
                x, y = int(item.center.x), int(item.center.y)
                image_cropped = cv2.resize(image[y - radius: y + radius,
                                           x - radius: x + radius],
                                           (40, 40))
                batch.append(image_cropped.astype(np.float32) / 255.0)
            # classify whole batch of images
            batch_conf = self._classifier.predict(batch)
            # only publish *one* candidate (with the highest confidence)
            maxconf, maxconf_index = np.max(batch_conf), np.argmax(batch_conf)
            if maxconf > self._classify_threshold:
                return ball_candidates[maxconf_index]

    def _find_candidates(self, image):
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cascade = cv2.CascadeClassifier('bla.xml')
        candidates = cascade.detectMultiScale(image_gray, 1.1, 1,
                                              minSize=(10, 10))
        # Todo: in welchem Format werden die Kandidaten hinterlegt?
        # Todo: wo wird nach Horizont gefiltert?
