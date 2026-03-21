^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package soccer_vision_3d_rviz_markers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2024-04-14)
------------------
* Fixed rviz marker indexes having duplicate markers of id zero (`#19 <https://github.com/ros-sports/soccer_vision_3d_rviz_markers/issues/19>`_)
* Rename __init__ file to __init__.py (`#14 <https://github.com/ros-sports/soccer_vision_3d_rviz_markers/issues/14>`_)
* Fix mistake in python3 typehint (`#14 <https://github.com/ros-sports/soccer_vision_3d_rviz_markers/issues/14>`_)
* Add parameter files for different robocup leagues (`#9 <https://github.com/ros-sports/soccer_vision_3d_rviz_markers/issues/9>`_)
* Contributors: Florian Vahl, Jan Gutsche, Kenji Brameld, abayomi, ijnek

0.0.1 (2022-07-08)
------------------
* Use a black and white ball mesh, so it looks better than a red sphere
* Simplify README by making it point to RTD.
* Add Ball, FieldBoundary, Goalpost, Marking, Obstacle, Robot conversions
* Update package description and license in setup.py
* Add rviz config for visualization demo
* Ensure DELETEALL marker is always included in the markerarray
* Removed foxy, galactic and humble ci
* Initial commit
* Contributors: Jan Gutsche, Kenji Brameld
