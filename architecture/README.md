This script generates an interactive overview of the software architecture based on information from the ROS packages and from a general definition in a .dia file.

To generate the overview just run update_architecture_svg.py in a terminal where you sourced your catkin workspace.


The architecture script works in the following steps
    1. export .dia file to .svg
    2. search the xml of the .svg for <rect> and <ellipse> to read which topics, services, actions and nodes are displayed
    3. get information about the nodes using their package.xml
    4. get information about topics and services by using command line calls on rosmsg and rossrv
    5. add onclick events on the corresponding xml tags to set the description taken from these
    6. set the color of nodes depending on their <status> in the package.xml
    7. put some html glue code with CSS magic around it
    8. ???
    9. profit

If you change the .dia file, take care of the following:
    - use rectangles for topics, services and actions
        - topics have solid lines, services have dashed lines, actions have dotted lines
        - first line has to be the topic name, second the message type
        - use full name of the message or service if its not from humanoid_league_msgs
            - i.e. geometry_msgs/Vector3        
    - use ellipses for nodes
    - beware of typos
    - source the devel/setup.* file of a complete bitbots_meta build before running the script
    - save the .dia file before running the script    
