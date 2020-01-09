\mainpage
\htmlinclude manifest.html
<a href="./index-msg.html">Msg/Src Documentation</a>

This ROS package includes usefull vision tools.

###image_extract.py
In the directory where you want your imagesets to be created run:
~~~
rosrun bitbots_vision_tools extract_from_rosbag.py -i <path_to_bag> -o <output directory> [-t <imagetopic>] [-n <every nth image will be extracted>]
~~~
Optional parameters are prompted when they are not specified.

Example:
~~~
rosrun bitbots_vision_tools extract_from_rosbag.py -i testdata.bag -o testdataset -t /image_raw -n 3
~~~
Will extract every third image from the testdata.bag on the /image_raw topic into the folder $PWD/testdataset
