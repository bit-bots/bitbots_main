import numpy as np
import cv2


class Classifier:
    def __init__(self, image, classifier, candidates):
        self._image = image
        self._input_candidates = candidates
        self._classifier = classifier
        self._classified_candidates = None
        self._top_candidate = None

    def get_classified_candidates(self):
        if self._classified_candidates is None:
            batch = list()
            if len(self._input_candidates) > 0:
                for item in self._input_candidates:
                    print(item)
                    image_cropped = cv2.resize(self._image[item[1]: item[1]+item[3],
                                               item[0]: item[0]+item[2]],
                                               (self._classifier.input_shape[0], self._classifier.input_shape[1]))
                    batch.append(image_cropped.astype(np.float32) / 255.0)
                # classify whole batch of images
                batch_conf = self._classifier.predict(batch)
                self._classified_candidates = [(self._input_candidates[i], batch_conf[i]) for i in range(len(batch_conf))]
            else:
                self._classified_candidates = list()
        return self._classified_candidates

    def get_top_candidate(self):
        if self._top_candidate is None and self.get_classified_candidates():
            maxconf_index = np.argmax(
                np.array([x[1] for x in self.get_classified_candidates()]))
            self._top_candidate = self._classified_candidates[maxconf_index]
            pass
        return self._top_candidate
