#!/bin/bash

roslaunch humanoid_league_transform transformer.launch &
roslaunch bitbots_localization transforms.launch &

for (( i=1; i<=$1; i++ ))
  do
    roslaunch bitbots_localization evaluation.launch &
    launch=$!
    python ~/robocup/bitbots_meta/bitbots_localization/scripts/statistics.py $i &
    statistics=$!
    sleep 10s && rosbag play ~/Dokumente/experiment_walking_3.bag --clock &
    rosbag=$!
    rosrun bitbots_localization feature_publisher.py &
    wait $rosbag
    kill -KILL $statistics
    kill -KILL $launch
  done
echo 'done'
