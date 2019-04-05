Animation Server
================

Der Animation Server kümmert sich um das Abspielen vorher aufgezeichneter Keyframe-Animationen. Er
wird von der HCM aufgerufen, wenn der Roboter gefallen ist und aufstehen möchte, kann aber auch vom
Behavior verwendet werden, beispielsweise um zu Schießen.

Der Animation Server ist ein ROS Action Server. Bei einer Action wird ähnlich zu einer Message eine
Nachricht an den Animation Server gesendet. Die Verbindung ist aber nicht nur einseitig, sondern
der Action Server sendet den Status der gerade durchgeführten Action sowie das Ergebnis
(erfolgreich oder nicht) an den Client zurück.

Animationen, die gerade abgespielt werden, können von der HCM unterbrochen werden. Das sorgt dafür,
dass der Roboter, fällt er beispielsweise beim Schießen, nicht den Schuss fertig abspielt, sondern
stattdessen aufsteht. Dies wird durch eine zusätzliche Flag in der Action umgesetzt, die angibt, ob
die Anfrage von der HCM stammt.

Animationen bestehen aus einer Abfolge von Keyframes. Jeder Keyframe ist dabei ein Abbild von
Motorpositionen zu einem bestimmten Zeitpunkt. Beim Abspielen einer Animation wird die
entsprechende Datei mit den aufgezeichneten Frames eingelesen und die Frames nacheinander in
vorgegebenen Abständen abgespielt. Um dabei eine flüssige Bewegung zu erhalten, wird mit einer
Frequenz von 200 Hz zwischen den Frames mithilfe von quintischen Splines interpoliert. Diese
Interpolation findet in Joint Space statt (also zwischen den Motorpositionen, nicht zwischen den
tatsächlichen Positionen der Gliedmaßen im kartesischen Raum, was auch denkbar wäre), was in erster
Linie historisch gewachsen und einfacher ist.

Manuell kann man Animationen mit `rosrun bitbots_animation_server run_animation <name>` starten.
Alle Animationen befinden sich im Paket `wolfgang_animations`.

Wenn das Abspielen einer Animation einmal nicht klappt, sollte geprüft werden, ob die HCM einen
anderen Robot State als Controlable oder Walking angibt, da nur in diesen beiden Status andere
Animationen abgespielt werden können.
