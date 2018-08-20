from bitbots_animation_server.spline_animator import SplineAnimator
import json
from bitbots_animation_server.animation import parse
from bitbots_animation_server.resource_manager import find_animation

with open(find_animation("walkready")) as fp:
    parsed_animation = parse(json.load(fp))

sanim = SplineAnimator(parsed_animation)
print(sanim.get_position(0))