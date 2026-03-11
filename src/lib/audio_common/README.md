# audio_capture

This repositiory provides a set of ROS 2 packages for audio. It provides a C++ version to capture and play audio data using PortAudio.

<div align="center">

[![License: MIT](https://img.shields.io/badge/GitHub-MIT-informational)](https://opensource.org/license/mit) [![GitHub release](https://img.shields.io/github/release/mgonzs13/audio_common.svg)](https://github.com/mgonzs13/audio_common/releases) [![Code Size](https://img.shields.io/github/languages/code-size/mgonzs13/audio_common.svg?branch=main)](https://github.com/mgonzs13/audio_common?branch=main) [![Last Commit](https://img.shields.io/github/last-commit/mgonzs13/audio_common.svg)](https://github.com/mgonzs13/audio_common/commits/main) [![GitHub issues](https://img.shields.io/github/issues/mgonzs13/audio_common)](https://github.com/mgonzs13/audio_common/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/mgonzs13/audio_common)](https://github.com/mgonzs13/audio_common/pulls) [![Contributors](https://img.shields.io/github/contributors/mgonzs13/audio_common.svg)](https://github.com/mgonzs13/audio_common/graphs/contributors) [![C++ Formatter Check](https://github.com/mgonzs13/audio_common/actions/workflows/cpp-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/audio_common/actions/workflows/cpp-formatter.yml?branch=main) [![Doxygen Deployment](https://github.com/mgonzs13/audio_common/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/audio_common/latest)

| ROS 2 Distro |                            Branch                            |                                                                                                         Build status                                                                                                          |                                                                  Docker Image                                                                   |
| :----------: | :----------------------------------------------------------: | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :---------------------------------------------------------------------------------------------------------------------------------------------: |
|   **Foxy**   | [`main`](https://github.com/mgonzs13/audio_common/tree/main) |       [![Foxy Build](https://github.com/mgonzs13/audio_common/actions/workflows/foxy-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/audio_common/actions/workflows/foxy-build-test.yml?branch=main)       |     [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-foxy-blue)](https://hub.docker.com/r/mgons/audio_common/tags?name=foxy)     |
| **Galactic** | [`main`](https://github.com/mgonzs13/audio_common/tree/main) | [![Galactic Build](https://github.com/mgonzs13/audio_common/actions/workflows/galactic-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/audio_common/actions/workflows/galactic-build-test.yml?branch=main) | [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-galactic-blue)](https://hub.docker.com/r/mgons/audio_common/tags?name=galactic) |
|  **Humble**  | [`main`](https://github.com/mgonzs13/audio_common/tree/main) |    [![Humble Build](https://github.com/mgonzs13/audio_common/actions/workflows/humble-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/audio_common/actions/workflows/humble-build-test.yml?branch=main)    |   [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-humble-blue)](https://hub.docker.com/r/mgons/audio_common/tags?name=humble)   |
|   **Iron**   | [`main`](https://github.com/mgonzs13/audio_common/tree/main) |       [![Iron Build](https://github.com/mgonzs13/audio_common/actions/workflows/iron-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/audio_common/actions/workflows/iron-build-test.yml?branch=main)       |     [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-iron-blue)](https://hub.docker.com/r/mgons/audio_common/tags?name=iron)     |
|  **Jazzy**   | [`main`](https://github.com/mgonzs13/audio_common/tree/main) |     [![Jazzy Build](https://github.com/mgonzs13/audio_common/actions/workflows/jazzy-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/audio_common/actions/workflows/jazzy-build-test.yml?branch=main)      |    [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-jazzy-blue)](https://hub.docker.com/r/mgons/audio_common/tags?name=jazzy)    |
|  **Kilted**  | [`main`](https://github.com/mgonzs13/audio_common/tree/main) |    [![Kilted Build](https://github.com/mgonzs13/audio_common/actions/workflows/kilted-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/audio_common/actions/workflows/kilted-build-test.yml?branch=main)    |   [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-kilted-blue)](https://hub.docker.com/r/mgons/audio_common/tags?name=kilted)   |
| **Rolling**  | [`main`](https://github.com/mgonzs13/audio_common/tree/main) |  [![Rolling Build](https://github.com/mgonzs13/audio_common/actions/workflows/rolling-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/audio_common/actions/workflows/rolling-build-test.yml?branch=main)   |  [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-rolling-blue)](https://hub.docker.com/r/mgons/audio_common/tags?name=rolling)  |

</div>

## Table of Contents

1. [Installation](#installation)
2. [Docker](#docker)
3. [Nodes](#nodes)
4. [Demos](#demos)

## Installation

```shell
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/audio_common.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Docker

You can create a docker image to test audio_common. Use the following command inside the directory of audio_common.

```shell
docker build -t audio_common .
```

After the image is created, run a docker container with the following command.

```shell
docker run -it --rm --device /dev/snd audio_common
```

## Nodes

### audio_capturer_node

Node to obtain audio data from a microphone and publish it into the `audio` topic.

<details>
<summary>Click to expand</summary>

#### Parameters

- **format**: Specifies the audio format to be used for capturing. Possible values are:

  - `1` (paFloat32 - 32-bit floating point)
  - `2` (paInt32 - 32-bit integer)
  - `8` (paInt16 - 16-bit integer)
  - `16` (paInt8 - 8-bit integer)
  - `32` (paUInt8 - 8-bit unsigned integer)

  Default: `8` (paInt16)

  The integer values correspond to PortAudio sample format flags.

- **channels**: The number of audio channels to capture. Typically, `1` for mono and `2` for stereo. Default: `1`

- **rate**: The sample rate that is how many samples per second should be captured. Default: `16000`

- **chunk**: The size of each audio frame. Default: `512`

- **device**: The ID of the audio input device. A value of `-1` indicates that the default audio input device should be used. Default: `-1`

- **frame_id**: An identifier for the audio frame. This can be useful for synchronizing audio data with other data streams. Default: `""`

#### ROS 2 Interfaces

- **audio**: Topic to publish the audio data captured from the microphone. Type: `audio_common_msgs/msg/AudioStamped`

</details>

### audio_player_node

Node to play the audio data obtained from the `audio` topic.

<details>
<summary>Click to expand</summary>

#### Parameters

- **channels**: The number of audio channels to play. Typically, `1` for mono and `2` for stereo. Default: `2`

  - The node automatically handles conversion between mono and stereo formats if needed.

- **device**: The ID of the audio output device. A value of `-1` indicates that the default audio output device should be used. Default: `-1`

#### ROS 2 Interfaces

- **audio**: Topic subscriber to get the audio data to be played. Type: `audio_common_msgs/msg/AudioStamped`

</details>

### music_node

Node to play music from audio files in `wav` format.

<details>
<summary>Click to expand</summary>

#### Parameters

- **chunk**: The size of each audio frame. Default: `2048`

- **frame_id**: An identifier for the audio frame. This can be useful for synchronizing audio data with other data streams. Default: `""`

#### ROS 2 Interfaces

- **audio**: Topic to publish the audio data from the files. Type: `audio_common_msgs/msg/AudioStamped`

- **music_play**: Service to play audio files. Type: `audio_common_msgs/srv/MusicPlay`

  - Parameters:
    - `audio`: Name of a built-in audio sample (e.g., "elevator")
    - `file_path`: Path to a custom WAV file (ignored if audio is specified)
    - `loop`: Boolean to indicate if the audio should loop. Default: `false`

- **music_stop**: Service to stop the currently playing music. Type: `std_srvs/srv/Trigger`

- **music_pause**: Service to pause the currently playing music. Type: `std_srvs/srv/Trigger`

- **music_resume**: Service to resume paused music. Type: `std_srvs/srv/Trigger`

</details>

### tts_node

Node to generate audio from text (TTS) using espeak.

<details>
<summary>Click to expand</summary>

#### Parameters

- **chunk**: The size of each audio frame. Default: `4096`

- **frame_id**: An identifier for the audio frame. This can be useful for synchronizing audio data with other data streams. Default: `""`

#### ROS 2 Interfaces

- **audio**: Topic publisher to send the audio data generated by the TTS. Type: `audio_common_msgs/msg/AudioStamped`

- **say**: Action to generate audio data from a text. Type: `audio_common_msgs/action/TTS`
  - Goal:
    - `text`: The text to convert to speech
    - `language`: The language to use for speech synthesis. Default: `"en"`
    - `volume`: The volume of the generated speech (0.0-1.0). Default: `1.0`
    - `rate`: The speech rate (1.0 is normal speed). Default: `1.0`
  - Feedback:
    - `audio`: The audio being currently played
  - Result:
    - `text`: The text that was converted to speech

</details>

## Demos

### Audio Capturer/Player

```shell
ros2 run audio_common audio_capturer_node
```

```shell
ros2 run audio_common audio_player_node
```

### TTS

```shell
ros2 run audio_common tts_node
```

```shell
ros2 run audio_common audio_player_node
```

```shell
ros2 action send_goal /say audio_common_msgs/action/TTS "{'text': 'Hello World'}"
```

Advanced TTS example with additional parameters:

```shell
ros2 action send_goal /say audio_common_msgs/action/TTS "{'text': 'Hello World', 'language': 'en-us', 'volume': 0.8, 'rate': 1.2}"
```

### Music Player

```shell
ros2 run audio_common music_node
```

```shell
ros2 run audio_common audio_player_node
```

Play a built-in sample:

```shell
ros2 service call /music_play audio_common_msgs/srv/MusicPlay "{audio: 'elevator'}"
```

Play a custom WAV file:

```shell
ros2 service call /music_play audio_common_msgs/srv/MusicPlay "{file_path: '/path/to/your/file.wav'}"
```

Play with looping enabled:

```shell
ros2 service call /music_play audio_common_msgs/srv/MusicPlay "{audio: 'elevator', loop: true}"
```

Control playback:

```shell
ros2 service call /music_pause std_srvs/srv/Trigger "{}"
ros2 service call /music_resume std_srvs/srv/Trigger "{}"
ros2 service call /music_stop std_srvs/srv/Trigger "{}"
```
