# bitbots_localization


Field distance maps use the full unsigned byte grayscale range, with dark pixels
representing field lines. Scoring normalizes the inverted intensity; published
occupancy grids convert it to the ROS percentage range. Out-of-field scores
remain configured as percentages. Regenerate maps from their saved generator inputs when changing field geometry.
