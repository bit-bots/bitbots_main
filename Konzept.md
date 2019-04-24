Konzept
=======

Aufteilen der Bewegung in mehrere Bestandteile, z.B.:

1. Fuß heben
2. Fuß zum Kick bewegen
3. Wieder stabil stehen

Während der gesamten Bewegung soll der Roboter stabil stehen.

Dazu haben wir zwei Ansätze:

1. In einer Control Loop wird nach jeder kleinen Bewegung der Schwerpunkt wieder über der
   Standfläche platziert.
2. Die integrierte Stabilisierungsmöglichkeit der BioIK wird genutzt
    1. Entweder auch in einem schnellen Zyklus, was möglicherweise unerwünschte Nebeneffekte haben
       könnte
    2. Nur bei den Kontrollpunkten (Fuß angehoben, Fuß vorne, …), aber kann zu Instabilität in der
       Bewegung führen.

Zur Berechnung der Bewegungen wollen wir quintische Splines verwenden.

Organisatorisches
=================

* Im Git nutzen wir Feature Branches
* Code soll gut kommentiert werden
* Issues benutzen, sowohl für Feature Requests, als auch für Bugs

Aktuelle Planung
================

Aktuell bekommen wir von außen eine Pose übergeben, die angibt, wohin sich der Fuß bewegen soll.
Später soll dies durch einen Punkt (Ballposition), einen Schusswinkel und ggf. eine Schussweite ersetzt werden.
Der Bewegungsablauf erhält dann einen zusätzlichen Zwischenpunkt (Fuß um eine dynamisch rekonfigurierbare Höhe
anheben und in die richtige Richtung drehen), den wir berechnen müssen, gefolgt vom eigentlichen Schuss mit
der der gewünschten Schussweite angepassten Geschwindigkeit. Danach erfolgt dann die Bewegung in die
Ursprungsposition.
