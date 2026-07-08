import colorsys


class HighContrastColorGenerator:
    """Generates RGB colors with a certain distance apart so that subsequent colors are visually distinct."""
    
    def __init__(self):
        # Golden ratio conjugate, ensures well-spaced hues
        self.hue_step = 0.61803398875
        self.hue = 0

    def __iter__(self):
        """Allows the class to be used as an iterable."""
        return self

    def __next__(self):
        """Generates the next high-contrast color."""
        self.hue = (self.hue + self.hue_step) % 1
        r, g, b = colorsys.hsv_to_rgb(self.hue, 1, 1)
        return (int(r * 255), int(g * 255), int(b * 255))

    def __call__(self):
        """Allows calling the instance directly to get the next color."""
        return next(self)


get_contrast_color = HighContrastColorGenerator()
"""A global instance to generate sequences of visually distinct colors.
"""

# TODO define some standard colors we want to use and use them in logging and node info sheets
