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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <portaudio.h>
#include <rclcpp/rclcpp.hpp>

#include "audio_common/audio_player_node.hpp"
#include "audio_common_msgs/msg/audio.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"

using namespace audio_common;
using std::placeholders::_1;

AudioPlayerNode::AudioPlayerNode() : Node("audio_player_node") {
  // Declare parameters
  this->declare_parameter<int>("channels", 2);
  this->declare_parameter<int>("device", -1);

  // Get parameters
  this->channels_ = this->get_parameter("channels").as_int();
  this->device_ = this->get_parameter("device").as_int();

  // Initialize PortAudio
  PaError err = Pa_Initialize();
  if (err != paNoError) {
    RCLCPP_ERROR(this->get_logger(), "PortAudio error: %s",
                 Pa_GetErrorText(err));
    throw std::runtime_error("Failed to initialize PortAudio");
  }

  // Subscription to audio topic
  auto qos_profile = rclcpp::SensorDataQoS();
  this->audio_sub_ =
      this->create_subscription<audio_common_msgs::msg::AudioStamped>(
          "audio", qos_profile,
          std::bind(&AudioPlayerNode::audio_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "AudioPlayer node started");
}

AudioPlayerNode::~AudioPlayerNode() {
  // Close all open streams and terminate PortAudio
  for (auto &stream_pair : this->stream_dict_) {
    Pa_StopStream(stream_pair.second);
    Pa_CloseStream(stream_pair.second);
  }
  Pa_Terminate();
}

void AudioPlayerNode::audio_callback(
    const audio_common_msgs::msg::AudioStamped::SharedPtr msg) {

  // Create a unique stream key based on format, rate, and channels
  std::string stream_key = std::to_string(msg->audio.info.format) + "_" +
                           std::to_string(msg->audio.info.rate) + "_" +
                           std::to_string(this->channels_);

  // Check if stream already exists, if not, create one
  if (this->stream_dict_.find(stream_key) == this->stream_dict_.end()) {
    PaStreamParameters outputParameters;
    outputParameters.device =
        (this->device_ >= 0) ? this->device_ : Pa_GetDefaultOutputDevice();
    outputParameters.channelCount = this->channels_;
    outputParameters.sampleFormat = msg->audio.info.format;
    outputParameters.suggestedLatency =
        Pa_GetDeviceInfo(outputParameters.device)->defaultHighOutputLatency;
    outputParameters.hostApiSpecificStreamInfo = nullptr;

    PaError err = Pa_OpenStream(&this->stream_dict_[stream_key], nullptr,
                                &outputParameters, msg->audio.info.rate, 1024,
                                paClipOff, nullptr, nullptr);

    if (err != paNoError) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open audio stream: %s",
                   Pa_GetErrorText(err));
      return;
    }
    Pa_StartStream(this->stream_dict_[stream_key]);
  }

  // Write audio from ROS 2 msg
  switch (msg->audio.info.format) {
  case paFloat32:
    this->write_data(msg->audio.audio_data.float32_data,
                     msg->audio.info.channels, msg->audio.info.chunk,
                     stream_key);
    break;

  case paInt32:
    this->write_data(msg->audio.audio_data.int32_data, msg->audio.info.channels,
                     msg->audio.info.chunk, stream_key);
    break;

  case paInt16:
    this->write_data(msg->audio.audio_data.int16_data, msg->audio.info.channels,
                     msg->audio.info.chunk, stream_key);
    break;

  case paInt8:
    this->write_data(msg->audio.audio_data.int8_data, msg->audio.info.channels,
                     msg->audio.info.chunk, stream_key);
    break;

  case paUInt8:
    this->write_data(msg->audio.audio_data.uint8_data, msg->audio.info.channels,
                     msg->audio.info.chunk, stream_key);
    break;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unsupported format");
    return;
  }
}

template <typename T>
void AudioPlayerNode::write_data(const std::vector<T> &input_data, int channels,
                                 int chunk, const std::string &stream_key) {

  std::vector<T> data; // Buffer for the actual data to write

  // Handle mono-to-stereo or stereo-to-mono conversions if necessary
  if (channels != this->channels_) {
    if (channels == 1 && this->channels_ == 2) {
      // Mono to stereo conversion
      data.resize(input_data.size() * 2);
      for (size_t i = 0; i < input_data.size(); ++i) {
        data[2 * i] = input_data[i];
        data[2 * i + 1] = input_data[i];
      }

    } else if (channels == 2 && this->channels_ == 1) {
      // Stereo to mono conversion
      data.resize(input_data.size() / 2);
      for (size_t i = 0; i < data.size(); ++i) {
        data[i] =
            static_cast<T>((input_data[2 * i] + input_data[2 * i + 1]) / 2);
      }
    }

  } else {
    // No conversion needed
    data = input_data;
  }

  // Make sure chunk size is correct for frames (not samples)
  if (data.size() < chunk * this->channels_) {
    RCLCPP_WARN(this->get_logger(),
                "Insufficient data (%ld) for requested chunk size (%d).",
                data.size(), chunk * this->channels_);
    return;
  }

  // Write in smaller blocks to reduce underrun risk
  size_t frames_written = 0;
  size_t total_frames = chunk;
  const size_t max_block = 1024;

  while (frames_written < total_frames) {
    size_t frames_to_write = std::min(max_block, total_frames - frames_written);
    PaError err = Pa_WriteStream(this->stream_dict_[stream_key],
                                 data.data() + frames_written * this->channels_,
                                 frames_to_write);

    if (err == paOutputUnderflowed) {
      RCLCPP_WARN(this->get_logger(),
                  "PortAudio underrun detected, retrying...");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue; // Try again this block

    } else if (err != paNoError) {
      RCLCPP_ERROR(this->get_logger(), "PortAudio write error: %s",
                   Pa_GetErrorText(err));
      break;
    }

    frames_written += frames_to_write;
  }
}
