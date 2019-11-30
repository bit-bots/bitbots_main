#!/bin/bash

roslaunch humanoid_league_transform transformer.launch &
roslaunch humanoid_league_localization transforms.launch &

for (( i=1; i<=$1; i++ ))
  do
    roslaunch humanoid_league_localization evaluation.launch &
    launch=$!
    python ~/robocup/bitbots_meta/humanoid_league_localization/scripts/statistics.py $i &
    statistics=$!
    sleep 10s && rosbag play ~/Dokumente/experiment_walking_3.bag --clock &
    rosbag=$!
    rosrun humanoid_league_localization feature_publisher.py &
    wait $rosbag
    kill -KILL $statistics
    kill -KILL $launch
  done
echo 'done'
