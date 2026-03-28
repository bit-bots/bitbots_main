// MIT License
//
// Copyright (c) 2024 Miguel Ángel González Santamarta
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef AUDIO_COMMON__AUDIO_PLAYER_NODE
#define AUDIO_COMMON__AUDIO_PLAYER_NODE

#include <memory>
#include <portaudio.h>
#include <rclcpp/rclcpp.hpp>

#include "audio_common_msgs/msg/audio_stamped.hpp"

namespace audio_common {

class AudioPlayerNode : public rclcpp::Node {
public:
  AudioPlayerNode();
  ~AudioPlayerNode() override;

private:
  // ROS 2 subscription for audio messages
  rclcpp::Subscription<audio_common_msgs::msg::AudioStamped>::SharedPtr
      audio_sub_;

  // PortAudio stream dictionary
  std::unordered_map<std::string, PaStream *> stream_dict_;

  // Parameters
  int channels_;
  int device_;

  // Methods
  void
  audio_callback(const audio_common_msgs::msg::AudioStamped::SharedPtr msg);
  template <typename T>
  void write_data(const std::vector<T> &data, int channels, int chunk,
                  const std::string &stream_key);
};

} // namespace audio_common

#endif