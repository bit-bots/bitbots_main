===========
Colorpicker
===========

Vorraussetzungen
================
Das Repository  Bit-Bots/wolves_colorpicker ist aus dem Gogs geclont worden.

Aufnehmen eines Colorspaces
===========================
1. Anschließen einer Kamera an den Computer
2. Hat der Computer eine eingebaute Kamera, setze in bitbots_meta/wolves_image_provider/config/camera_settings.yaml unter camera_v4l den Configwert device auf /dev/video1
3. Starte den Colorpicker: roslaunch colorpicker colorpicker_startup.launch
4. Wechsle in das Fenster, wo man das Bild der Kamera sieht
5. Vergrößere die Größe des blauen Markierungsblockes mittels der Leiste unten im Fenster. Es erscheint im Bild ein blauer Kasten, wenn man mit der Maus drauf geht. Der rote Kasten ist irrelevant.
6. Markiere das Spielfeld durch Klicken auf die grünen Flächen im Bild. Dabei wird immer der Bereich ausgewählt, der in dem blauen Kasten zu sehen ist. Der Kasten sollte ausschließlich grün, also nur Gras enthalten.
7. Es erscheinen grüne Punkte auf den Bild. Wiederhole Schritt 6 solange bis das ganze Feld, außer die Linine, von grünen Punkten übersät ist. Achte darauf, das Feld aus verschiedenen Blickwinkeln mit der Kamera aufzunehmen.
8. Wechsele zurück in die Shell und drücke w zum Speichern


Was tun mit dem aufgenommenen Colorspace?
==========================================
1. Der Colorspace ist gespeichert in der Datei ~/.ros/yamlColor.yaml
2. Verschiebe die Datei nach bitbots_meta/bitbots_vision/bitbots_vision/config/color_spaces
3. Benenne die Datei sinnvoll (z.B. Veranstaltungsname)
4. Interpolation-tool ausführen: ./bitbots_meta/bitbots_vision/bitbots_vision_tools/scripts/colospacetool.py -y path_to_inputfile -o
5. Passe den Configparameter field_color_detector_path in bitbots_vision/bitbots_vision/config/visionparams.yaml an
6. Pushen
