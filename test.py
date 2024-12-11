from random import uniform

import matplotlib
import matplotlib.patches
from bbpprs import ObstacleMap, ObstacleMapConfig, Robot
from matplotlib import pyplot as plt

config = ObstacleMapConfig(dilate=0.1)


def random_omap(config, obstacles):
    l = []
    for _ in range(obstacles):
        x = uniform(1.0, 9.0)
        y = uniform(1.0, 9.0)
        r = uniform(0.2, 2.0)
        l.append(Robot((x, y), r))
    return ObstacleMap(config, l, None)


def debug_omap(config):
    l = []
    l.append(Robot((1.0, 4.0), 1.0))
    l.append(Robot((4.2, 4.2), 1.0))
    l.append(Robot((5.0, 2.0), 1.4))
    l.append(Robot((10.0, 10.0), 0.5))
    return ObstacleMap(config, l, None)


"""
def random_omaps(config, n):
    l = []
    for _ in range(n):
        l.append(random_omap(config, 3))
    return l

omaps = random_omaps(config, 100)

_ = [om.shortest_path(start=(0.0, 0.0), goal=(10.0, 10.0)) for om in omaps]
"""

omap = debug_omap(config)
path = omap.shortest_path(start=(0.0, 0.0), goal=(10.0, 10.0))
print(path)
fig, ax = plt.subplots(1)
ax.axis([0, 11, 0, 11])
for obstacle in omap.robots:
    ax.add_patch(matplotlib.patches.RegularPolygon(obstacle.center, 12, radius=obstacle.radius))
ax.plot([i[0] for i in path], [i[1] for i in path])
print(str(omap.ball), [str(bot) for bot in omap.robots])
# plt.plot(path)
plt.show()
