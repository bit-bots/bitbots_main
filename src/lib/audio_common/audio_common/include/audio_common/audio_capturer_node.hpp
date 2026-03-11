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

#ifndef AUDIO_COMMON__AUDIO_CAPTURER_NODE
#define AUDIO_COMMON__AUDIO_CAPTURER_NODE

#include <memory>
#include <portaudio.h>
#include <rclcpp/rclcpp.hpp>

#include "audio_common_msgs/msg/audio_stamped.hpp"

namespace audio_common {

class AudioCapturerNode : public rclcpp::Node {
public:
  AudioCapturerNode();
  ~AudioCapturerNode() override;

  void work();

private:
  PaStream *stream_;
  int format_;
  int channels_;
  int rate_;
  int chunk_;
  std::string frame_id_;

  rclcpp::Publisher<audio_common_msgs::msg::AudioStamped>::SharedPtr audio_pub_;

  // Methods
  template <typename T> std::vector<T> read_data();
};

} // namespace audio_common

#endif