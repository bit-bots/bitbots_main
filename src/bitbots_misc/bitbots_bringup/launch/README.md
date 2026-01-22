### How to record audio to ROS Bag

Launch these commands in parallel:

```bash
ros2 launch bitbots_bringup audio.launch
ros2 bag record /audio/audio /audio/audio_info
```

### How to play audio from ROS Bag

Launch these commands in parallel:

```bash
ros2 bag play <bagfile> --loop
ros2 launch audio_play play.launch.xml format:=wave sample_rate:=10000
```
