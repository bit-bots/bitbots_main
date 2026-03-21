## ROS service for calling BioIK from Python or Java
- ROS message definitions for BioIK goal types
- BioIK service node
- New /bio_ik/get_bio_ik service with full support for new BioIK goal types
- Legacy /bio_ik/get_position_ik interface for compatibility

### Running the examples
- Clone https://github.com/TAMS-Group/bio_ik, https://github.com/TAMS-Group/bioik_pr2, and this repository
- Launch the PR2+BioIK demo config: `roslaunch pr2_moveit_bio_ik demo.launch`
- Launch bioik_service: `rosrun bio_ik_service bio_ik_service`
- Launch one of the demo scripts, e.g.: `rosrun bio_ik_service_examples get_bio_ik.py`

#### See [./bio_ik_service_examples/scripts](./bio_ik_service_examples/scripts) for more information on how to use the BioIK service...
