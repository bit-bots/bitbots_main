"""Native controls for atomic runtime updates of referee parameters."""

import time

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDockWidget, QDoubleSpinBox, QFormLayout, QLabel, QPushButton, QWidget
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from bitbots_auto_referee.config import LIVE_PARAMETERS


class RefereeSettings:
    def __init__(self, node, window):
        self.client = AsyncParameterClient(node, f"{node.get_namespace().rstrip('/')}/auto_referee")
        self.future = None
        self.sent_at = None
        self.initialized = False
        self.dirty = False
        dock = QDockWidget("AutoRef-Einstellungen", window)
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        body = QWidget(dock)
        form = QFormLayout(body)
        self.inputs = {}
        for name, label, unit in zip(
            LIVE_PARAMETERS,
            ("Anhaltender Kontakt", "Einzelkontakt", "Set-Play-Mindestabstand zum Ball"),
            (" N", " N", " m"),
            strict=True,
        ):
            control = QDoubleSpinBox(body)
            control.setDecimals(4)
            control.setRange(0.0001, 1e9)
            control.setSingleStep(0.1)
            control.setSuffix(unit)
            control.valueChanged.connect(self._edited)
            form.addRow(label, control)
            self.inputs[name] = control
        explanation = QLabel(
            "Höhere Kraftschwellen machen die Pushing-Erkennung unempfindlicher.\n"
            "Einzelkontakt erfordert zusätzlich eine Destabilisierung.\n"
            "Anhaltender Kontakt darf die Einzelkontakt-Schwelle nicht überschreiten.\n"
            "Der Mindestabstand gilt für Gegner zwischen Körpergrenze und Ballmitte.",
            body,
        )
        explanation.setWordWrap(True)
        form.addRow(explanation)
        self.current = QLabel("Warte auf aktuelle Schwellwerte …", body)
        self.current.setWordWrap(True)
        form.addRow(self.current)
        self.apply = QPushButton("Übernehmen", body)
        self.apply.setEnabled(False)
        self.apply.clicked.connect(self._submit)
        form.addRow(self.apply)
        self.status = QLabel("", body)
        self.status.setWordWrap(True)
        form.addRow(self.status)
        dock.setWidget(body)
        window.addDockWidget(Qt.BottomDockWidgetArea, dock)

    def _edited(self, _value):
        self.dirty = True

    def _submit(self):
        sustained, single = (self.inputs[name].value() for name in LIVE_PARAMETERS[:2])
        if sustained > single:
            self.status.setText("Anhaltender Kontakt darf die Einzelkontakt-Schwelle nicht überschreiten.")
            return
        self.future = self.client.set_parameters_atomically(
            [Parameter(name, value=self.inputs[name].value()) for name in LIVE_PARAMETERS]
        )
        self.sent_at = time.monotonic()
        self.apply.setEnabled(False)
        self.status.setText("Änderung wird bestätigt …")

    def refresh(self, snapshot, stale):
        if self.future is not None:
            if self.future.done():
                try:
                    result = self.future.result().result
                    if result.successful:
                        self.status.setText("Übernommen. Die neuen Einstellungen sind aktiv.")
                        self.dirty = False
                    else:
                        self.status.setText(f"Abgelehnt: {result.reason}")
                except Exception as error:
                    self.status.setText(f"Änderung fehlgeschlagen: {error}")
                self.future = None
            elif time.monotonic() - self.sent_at > 5:
                self.future.cancel()
                self.future = None
                self.status.setText("Keine Bestätigung. Aktive Werte prüfen und gegebenenfalls erneut übernehmen.")
        values = (snapshot or {}).get("runtime_parameters", {})
        available = all(name in values for name in LIVE_PARAMETERS)
        if available:
            sustained, single, distance = (values[name] for name in LIVE_PARAMETERS)
            self.current.setText(
                f"Aktiv: anhaltend {sustained:g} N · einzeln {single:g} N · Set Play {distance:g} m"
            )
            if not self.initialized or (not self.dirty and self.future is None):
                for name, control in self.inputs.items():
                    control.blockSignals(True)
                    control.setValue(values[name])
                    control.blockSignals(False)
                self.initialized = True
        self.apply.setEnabled(available and not stale and self.future is None and self.client.services_are_ready())
