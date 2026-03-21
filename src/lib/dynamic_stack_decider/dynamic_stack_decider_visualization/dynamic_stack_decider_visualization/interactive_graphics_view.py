# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QTransform
from PyQt5.QtWidgets import QGraphicsView


# ruff: noqa: N802
class InteractiveGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("InteractiveGraphicsView")

        self._last_pan_point = None
        self._last_scene_center = None

    def mousePressEvent(self, mouse_event):
        self._last_pan_point = mouse_event.pos()
        self._last_scene_center = self._map_to_scene_f(QRectF(self.frameRect()).center())
        self.setCursor(Qt.ClosedHandCursor)

    def mouseReleaseEvent(self, mouse_event):
        self.setCursor(Qt.OpenHandCursor)
        self._last_pan_point = None

    def mouseMoveEvent(self, mouse_event):
        if self._last_pan_point is not None:
            delta_scene = self.mapToScene(mouse_event.pos()) - self.mapToScene(self._last_pan_point)
            if not delta_scene.isNull():
                self.centerOn(self._last_scene_center - delta_scene)
                self._last_scene_center -= delta_scene
            self._last_pan_point = mouse_event.pos()
        QGraphicsView.mouseMoveEvent(self, mouse_event)

    def wheelEvent(self, wheel_event):
        if wheel_event.modifiers() == Qt.NoModifier:
            try:
                delta = wheel_event.angleDelta().y()
            except AttributeError:
                delta = wheel_event.delta()
            delta = max(min(delta, 480), -480)
            mouse_before_scale_in_scene = self.mapToScene(wheel_event.pos())

            scale_factor = 1 + (0.2 * (delta / 120.0))
            scaling = QTransform(scale_factor, 0, 0, scale_factor, 0, 0)
            self.setTransform(self.transform() * scaling)

            mouse_after_scale_in_scene = self.mapToScene(wheel_event.pos())
            center_in_scene = self.mapToScene(self.frameRect().center())
            self.centerOn(center_in_scene + mouse_before_scale_in_scene - mouse_after_scale_in_scene)

            wheel_event.accept()
        else:
            QGraphicsView.wheelEvent(self, wheel_event)

    def _map_to_scene_f(self, point_f):
        point = point_f.toPoint()
        if point_f.x() == point.x() and point_f.y() == point.y():
            # map integer coordinates
            return self.mapToScene(point)
        elif point_f.x() == point.x():
            # map integer x and decimal y coordinates
            point_a = self.mapToScene((point_f + QPointF(0, -0.5)).toPoint())
            point_b = self.mapToScene((point_f + QPointF(0, 0.5)).toPoint())
            return (point_a + point_b) / 2.0
        elif point_f.y() == point.y():
            # map decimal x  and integer y and coordinates
            point_a = self.mapToScene((point_f + QPointF(-0.5, 0)).toPoint())
            point_b = self.mapToScene((point_f + QPointF(0.5, 0)).toPoint())
            return (point_a + point_b) / 2.0
        else:
            # map decimal coordinates
            point_a = self.mapToScene((point_f + QPointF(-0.5, -0.5)).toPoint())
            point_b = self.mapToScene((point_f + QPointF(-0.5, 0.5)).toPoint())
            point_c = self.mapToScene((point_f + QPointF(0.5, -0.5)).toPoint())
            point_d = self.mapToScene((point_f + QPointF(0.5, 0.5)).toPoint())
            return (point_a + point_b + point_c + point_d) / 4.0
