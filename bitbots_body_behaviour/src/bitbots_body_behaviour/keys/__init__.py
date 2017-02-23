# -*- coding:utf-8 -*-
"""
Keys
====

.. moduleauthor:: Nils Rokita <0rokita@informatik.uni-hamburg.de>

History:
''''''''

* 4.3.14: Created (Nils Rokita)

* 11.04.14: Refactoring (Sheepy Keßler)

* 05.08.14: Refactoring for better Doku (Marc Bestmann)

* 17.07.15: Removed Trainer Key

Hier sind alle ModulDependenzies und Felder die es gibt aufgeführt
Sie sollten mit dem Präfix DATA_KEY für einen Key und DATA_VALUE für einen
value beginnen

.. automodule:: bitbots.modules.keys.motion_keys

.. automodule:: bitbots.modules.keys.network_keys

.. automodule:: bitbots.modules.keys.vision_keys

.. automodule:: bitbots.modules.keys.post_processing_keys

.. automodule:: bitbots.modules.keys.walking_keys

.. automodule:: bitbots.modules.keys.world_model_keys



Restliche Keys
--------------

Jetzt folgen die Keys, für die keine Gruppe besteht
"""

from bitbots_body_behaviour.keys.network_keys import *
from bitbots_body_behaviour.keys.vision_keys import *
from bitbots_body_behaviour.keys.post_processing_keys import *
from bitbots_body_behaviour.keys.motion_keys import *
from bitbots_body_behaviour.keys.world_model_keys import *
from bitbots_body_behaviour.keys.walking_keys import *

DATA_KEY_CONFIG = "Config"
""" Name der Config """

DATA_KEY_DUTY = "Duty"
""" Die Duty des Roboters, also z.B. TeamPlayer, Striker, Goalie, ... """

DATA_KEY_GIVE_UP_GOAL = "Behaviour.GivenUpGoal"

DATA_KEY_MOVING_X = "Moving.X"
""" TODO KOMMENTAR """

DATA_KEY_MOVING_Y = "Moving.Y"
""" TODO KOMMENTAR """

DATA_KEY_MOVING_DIRECTION = "Moving.Direction"
""" TODO KOMMENTAR """

DATA_KEY_POSITION = "Position"
""" TODO KOMMENTAR """

# ab hier neue keys die noch sortiert werden müssen

DATA_KEY_ANIMATION = "Animation"
""" Abzuspielende Animation """

DATA_KEY_DATA_CAMERA = "DataCamera"
""" Ob die datenkamera an ist, wir uns also bilder ansehen"""

DATA_KEY_ANY_WHOLE_GOAL_LAST_SEEN = "AnyWholeGoalLastSeen"
""" TODO DOKU"""

DATA_KEY_HORIZON_OBSTACLES = "horizon_obstacles"
""" TODO DOKU"""

DATA_KEY_TRANSFORMER_UPDATED = "transformerUpdated"
""" TODO DOKU """

DATA_KEY_IGNORE_MASQ_HITS = "IgnoreMasqHits"
""" TDO ODOKU """
