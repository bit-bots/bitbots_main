# Copyright (c) 2022 Kenji Brameld
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ipm_library.utils import plane_general_to_point_normal
import numpy as np
from shape_msgs.msg import Plane


def test_plane_general_to_point_normal_1():
    plane = Plane(coef=[2.0, 0.0, 0.0, -4.0])  # x = 2.0 (2.0x + 0y + 0z - 4.0 = 0)
    (n, p) = plane_general_to_point_normal(plane)
    assert np.allclose(n, [2.0, 0.0, 0.0], rtol=0.0001)
    assert np.allclose(p, [1.0, 0.0, 0.0], rtol=0.0001)


def test_plane_general_to_point_normal_2():
    plane = Plane(coef=[0.0, 0.0, -1.0, -1.0])  # z = -1.0 (0x + 0y - 1.0z - 1.0 = 0)
    (n, p) = plane_general_to_point_normal(plane)
    assert np.allclose(n, [0.0, 0.0, -1.0], rtol=0.0001)
    assert np.allclose(p, [0.0, 0.0, -1.0], rtol=0.0001)
