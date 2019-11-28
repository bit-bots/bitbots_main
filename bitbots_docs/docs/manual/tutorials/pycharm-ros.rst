============================
PyCharm mit ROS Integration
============================

Paketpfad einrichten
====================
Hiermit werden ROS-Pakete zum PYTHONPATH hinzugefügt, wodurch diese von PyCharm indexiert werden und somit
Autovervolständigung möglich wird.
Wenn `ROS Sourcen` angewendet wird, sollte dies nicht nötig sein.

1. Settings --> Project:bitbots_meta --> Project Interpreter
2. Zahnrad oben rechts neben dem Interpreter --> Show All
3. Unterster Button auf der rechten Seite ("Show paths for the selected interpreter")
4. Add --> `/opt/ros/melodic/lib/python2.7/dist-packages`
5. Add --> `<catkin_ws>/devel/lib/python2.7/dist-packages`

ROS Sourcen
===========
Hierdurch wird ROS beim starten von PyCharm gesourced wodurch alle ROS-Commands und Pfade automatisch bekannt werden.
PyCharm weiß somit genauso viel wie die Shell und kann theoretisch auch genauso viel machen.

1. ::

    cp /usr/share/applications/pycharm-professional.desktop ~/.local/share/applications/pycharm-with-ros.desktop

2. ::

    vim ~/.local/share/applications/pycharm-with-ros.desktop

3. ::

    Name ändern, da sonst der globale PyCharm starter überschrieben wird

4. ::

    Exec=bash -c "source <catkin_ws>/devel/setup.bash; /usr/bin/pycharm %f"


Launch-Dateien anpassen
=======================
Um Nodes effektiv mit PyCharm zu testen/debuggen kann ein Argument `depends_only` zur Launch-Datei hinzugefügt werden,
wodurch nur konditional die zu testende Node gestartet wird.

Nachdem dann extern mit `roslaunch` diese Datei gelauncht wurde kann PyCharm die eigentliche Node einfach
ausführen und debuggen.
