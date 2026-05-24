^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package zed_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

5.2.1 (2026-03-02)
------------------
* Remove unused URDF files and associated references from CMakeLists and README
  * Mesh files are now in the `zed_description` package available via APT
* Improve formatting and clarity in CONTRIBUTING.md
* Fix CONTRIBUTING
* Fix README
* Contributors: Walter Lucetti

5.1.1 (2025-10-22)
------------------
* Add ZED X HDR CAD models
* Update 3D models with smaller files
* Contributors: Walter Lucetti

5.1.0 (2025-10-16)
------------------
* Add SaveAreaMemory service definition
* Contributors: Walter Lucetti

5.0.1 (2025-05-26)
------------------
* Add SaveAreaMemory custom service
* Contributors: Walter Lucetti

5.0.0 (2025-03-28)
------------------
* Add new SvoStatus topic
* Add new HealthStatusStamped topic
* Add new SetSvoFrame service

4.2.5 (2025-03-25)
------------------
* Update Object.msg
* Update README.md

Old releases
------------------
* Changed the name of the package `zed_interfaces` to `zed_msgs` to match the ROS2 naming convention
* Add ZED X One mesh
* Update packages version
* Update cmake_minimum_required
* Fix to support Iron
* Fix Foxy building
* Fix dependencies
* Add ZED X One STL
* Fix PosTrackStatusMsg
* Update PosTrackStatus.msg
* Add new GNSS status message and update PosTrackStatus
*  Add mag heading status
*  Update PosTrackStatus.msg
*  Fix typo in enum for SEARCHING_FLOOR_PLANE
* Improve install info
* Update SetPose srv description
* Add new PosTrackStatus message type
* Add support for new BODY formats
*  Add zedx and zedxm meshes
* Remove annoying build log messages
* Fix LINT test
* Add CONTRIBUTING rules
* Add `setRoi` service
* Add ROS2 distro check
* Add PlaneStamped message
* Add DepthInfoStamped.msg
* Add support for BODY_FORMAT::POSE_34
* Update Object topic
* Add msg, srv, and meshes files
* Initial commit
