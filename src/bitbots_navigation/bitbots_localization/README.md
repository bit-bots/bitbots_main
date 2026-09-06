# bitbots_localization


Field distance maps use the full unsigned byte grayscale range, with dark pixels
representing field lines. Scoring normalizes the inverted intensity; published
occupancy grids convert it to the ROS percentage range. Out-of-field scores
remain configured as percentages. Regenerate maps with the field-map script
under `scripts/field_maps` when changing their generator inputs.
