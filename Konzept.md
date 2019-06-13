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
Stand 05.06.2019
Einige der letzten ToDos sind erledigt:
* Wir publishen nun den Basefootprint korrekt
* Der Fuß wird nun entsprechend der Schussrichtung gedreht (bei Winkeln, die mehr als 45° zur Seite gehen, wird mit der Seite des Fußes geschossen)
* Die HCM erkennt einen Kick-Abbruch nun, indem als neues Kriterium, ob der Kick noch ausgeführt wird, die Zeit seit dem letzten Feedback der KickNode geprüft wird.
* Der Kick ist nun insgesamt durch deutlich mehr Goals für die BioIK stabiler. Unter anderem geben wir nun vor, dass der Roboter den Oberkörper nur noch seitwärts verlagern darf und ihn dabei gerade halten soll.
* Die Fuß-Auswahl erfolgt nun nicht mehr ausschließlich anhand des Winkelkriteriums. Stattdessen wird außerhalb eines konfigurierbar breiten Korridors immer mit dem jeweils näheren Fuß geschossen.

Noch zu erledigen bleibt:
* self-collision verhindern: Durch die flexible Zielauswahl kommt es bei einigen Ballpositionen und Schusswinkeln derzeit zu einer Selbstkollision der Beine. Dies soll mindestens kurz vorher erkannt werden. Falls der Roboter sich schon bewegt hat, soll er möglichst den gleichen Weg zurück in die Ausgangsposition nehmen.
* Fuß-Auswahl optimieren: Die bisherigen Kriterien erscheinen in manchen Situationen noch nicht optimal. Dies soll evaluiert und ggf. weiter verbessert werden.
* Die Bewegungsgeschwindigkeit muss noch eingebaut werden.
* Wir müssen womöglich den ActionServer überarbeiten, da er im Moment jedes Ziel ohne Überprüfung akzeptiert.

Stand 29.05.2019
Also ToDos bleiben noch:
* Ende des Schusses soll in der HCM erkennbar sein, auch falls der Schuss abgebrochen wird
* Basefootprint muss gepublished werden
* Fuß soll entsprechend der Schussrichtung gedreht werden
* Kicking Time und ähnliche Parameter sollen rausfallen und durch eine konfigurierbare einhaltliche Geschwindigkeit für nicht Kick-Bewegungen ersetzt werden
* Schuss soll entsprechend der tatsächlichen Möglichkeiten begrenzt werden, unrealistische Ziele sollen abgelehnt werden

Stand 27.05.2019
Wir bekommen nun eine Ballposition und eine Schussrichtung (als Vektor) übergeben. Wir rechnen dann in der Engine einen Punkt in entgegengesetzter Richtung (mit dynamic reconfigurable distance) aus, zu dem der Fuß bewegt wird. Von dort wird dann der Schuss ausgeführt.
Zusätzlich führen wir aktuell Stabilisierung ein, was einige der Splines ersetzt: Wir brauchen Splines jetzt nur noch für den Kicking Foot. Alles andere kann über die Stabilisierung der BioIK gelöst werden. Diese benötigt dafür einen Punkt, über dem balanciert werden soll. Um die Bewegung am Anfang flüssig zu gestalten, soll dieser Punkt zunächst in der Mitte unter dem Roboter liegen und dann zu einem Fuß verschoben werden. Der genaue Punkt hierfür muss noch gefunden werden.

Alter Stand 24.04.2019
Aktuell bekommen wir von außen eine Pose übergeben, die angibt, wohin sich der Fuß bewegen soll.
Später soll dies durch einen Punkt (Ballposition), einen Schusswinkel und ggf. eine Schussweite ersetzt werden.
Der Bewegungsablauf erhält dann einen zusätzlichen Zwischenpunkt (Fuß um eine dynamisch rekonfigurierbare Höhe
anheben und in die richtige Richtung drehen), den wir berechnen müssen, gefolgt vom eigentlichen Schuss mit
der der gewünschten Schussweite angepassten Geschwindigkeit. Danach erfolgt dann die Bewegung in die
Ursprungsposition.
