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
#include <portaudio.h>
#include <rclcpp/rclcpp.hpp>

#include "audio_common/audio_capturer_node.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"

using namespace audio_common;

AudioCapturerNode::AudioCapturerNode() : Node("audio_capturer_node") {

  // Declare parameters with default values
  this->declare_parameter<int>("format", paInt16);
  this->declare_parameter<int>("channels", 1);
  this->declare_parameter<int>("rate", 16000);
  this->declare_parameter<int>("chunk", 512);
  this->declare_parameter<int>("device", -1);
  this->declare_parameter<std::string>("frame_id", "");

  // Get parameters
  this->format_ = this->get_parameter("format").as_int();
  this->channels_ = this->get_parameter("channels").as_int();
  this->rate_ = this->get_parameter("rate").as_int();
  this->chunk_ = this->get_parameter("chunk").as_int();
  int device = this->get_parameter("device").as_int();
  this->frame_id_ = this->get_parameter("frame_id").as_string();

  // Initialize PortAudio
  PaError err = Pa_Initialize();
  if (err != paNoError) {
    RCLCPP_ERROR(this->get_logger(), "PortAudio error: %s",
                 Pa_GetErrorText(err));
    throw std::runtime_error("Failed to initialize PortAudio");
  }

  PaStreamParameters inputParameters;
  inputParameters.device = (device >= 0) ? device : Pa_GetDefaultInputDevice();
  inputParameters.channelCount = this->channels_;
  inputParameters.sampleFormat = this->format_;
  inputParameters.suggestedLatency =
      Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
  inputParameters.hostApiSpecificStreamInfo = nullptr;

  err = Pa_OpenStream(&this->stream_, &inputParameters,
                      nullptr, // output parameters (not used)
                      this->rate_, this->chunk_, paClipOff, nullptr, nullptr);

  if (err != paNoError) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open audio stream: %s",
                 Pa_GetErrorText(err));
    throw std::runtime_error("Failed to open PortAudio stream");
  }

  err = Pa_StartStream(this->stream_);
  if (err != paNoError) {
    RCLCPP_ERROR(this->get_logger(), "Failed to start audio stream: %s",
                 Pa_GetErrorText(err));
    throw std::runtime_error("Failed to start PortAudio stream");
  }

  this->audio_pub_ =
      this->create_publisher<audio_common_msgs::msg::AudioStamped>(
          "audio", rclcpp::SensorDataQoS());

  RCLCPP_INFO(this->get_logger(), "AudioCapturer node started");
}

AudioCapturerNode::~AudioCapturerNode() {
  Pa_StopStream(this->stream_);
  Pa_CloseStream(this->stream_);
  Pa_Terminate();
}

void AudioCapturerNode::work() {
  while (rclcpp::ok()) {

    auto msg = audio_common_msgs::msg::AudioStamped();
    msg.header.frame_id = this->frame_id_;
    msg.header.stamp = this->get_clock()->now();

    switch (this->format_) {
    case paFloat32: {
      msg.audio.audio_data.float32_data = this->read_data<float>();
      break;
    }
    case paInt32: {
      msg.audio.audio_data.int32_data = this->read_data<int32_t>();
      break;
    }
    case paInt16: {
      msg.audio.audio_data.int16_data = this->read_data<int16_t>();
      break;
    }
    case paInt8: {
      msg.audio.audio_data.int8_data = this->read_data<int8_t>();
      break;
    }
    case paUInt8: {
      msg.audio.audio_data.uint8_data = this->read_data<uint8_t>();
      break;
    }
    default:
      RCLCPP_ERROR(this->get_logger(), "Unsupported format");
      continue;
    }

    msg.audio.info.format = this->format_;
    msg.audio.info.channels = this->channels_;
    msg.audio.info.chunk = this->chunk_;
    msg.audio.info.rate = this->rate_;

    this->audio_pub_->publish(msg);
  }
}

template <typename T> std::vector<T> AudioCapturerNode::read_data() {
  std::vector<T> data(this->chunk_ * this->channels_);
  Pa_ReadStream(this->stream_, data.data(), this->chunk_);
  return data;
}
