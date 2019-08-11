import numpy as np
import cv2
from .candidate import Candidate, CandidateFinder


class ClassifierHandler(CandidateFinder):
    def __init__(self, classifier):
        # type: (LiveClassifier) -> None
        self._image = None
        self._input_candidates = None
        self._classifier = classifier
        self._classified_candidates = None
        self._sorted_candidates = None
        self._top_candidate = None

    def set_image(self, image, candidates):
        self._image = image
        self._input_candidates = candidates
        self._classified_candidates = None
        self._sorted_candidates = None
        self._top_candidate = None

    def get_candidates(self):
        if self._classified_candidates is None:
            batch = list()
            if self._input_candidates:
                for item in self._input_candidates:
                    image_cropped = cv2.resize(self._image[item.get_upper_left_y(): item.get_upper_left_y()+item.get_height(),
                                               item.get_upper_left_x(): item.get_upper_left_x()+item.get_width()],
                                               (self._classifier.input_shape[0], self._classifier.input_shape[1]))
                    batch.append(image_cropped.astype(np.float32) / 255.0)
                # classify whole batch of images
                batch_conf = self._classifier.predict(batch)
                for i in range(len(batch_conf)):
                    self._input_candidates[i].rating=batch_conf[i]
                self._classified_candidates = self._input_candidates
            else:
                self._classified_candidates = list()
        return self._classified_candidates

    def compute(self):
        """
        Method to satisfy the interface
        """
        pass

    def get_top_candidates(self, count=1):
        if self._sorted_candidates is None:
            self._sorted_candidates = sorted(self.get_candidates(), key=lambda x: x.rating)
        return self._sorted_candidates[0:count]

