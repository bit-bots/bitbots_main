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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <audio_common_msgs/msg/audio_stamped.hpp>
#include <audio_common_msgs/srv/music_play.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "audio_common/music_node.hpp"
#include "audio_common/wave_file.hpp"

using namespace audio_common;
using std::placeholders::_1;
using std::placeholders::_2;

MusicNode::MusicNode()
    : Node("music_node"), pause_music_(false), stop_music_(false),
      audio_loop_(false), is_thread_alive_(false) {

  // Parameters
  this->declare_parameter<int>("chunk", 2048);
  this->declare_parameter<std::string>("frame_id", "");

  this->chunk_ = this->get_parameter("chunk").as_int();
  this->frame_id_ = this->get_parameter("frame_id").as_string();

  // Publisher
  this->player_pub_ =
      this->create_publisher<audio_common_msgs::msg::AudioStamped>(
          "audio", rclcpp::SensorDataQoS());

  // Services
  this->play_service_ = this->create_service<audio_common_msgs::srv::MusicPlay>(
      "music_play", std::bind(&MusicNode::play_callback, this, _1, _2));
  this->stop_service_ = this->create_service<std_srvs::srv::Trigger>(
      "music_stop", std::bind(&MusicNode::stop_callback, this, _1, _2));
  this->pause_service_ = this->create_service<std_srvs::srv::Trigger>(
      "music_pause", std::bind(&MusicNode::pause_callback, this, _1, _2));
  this->resume_service_ = this->create_service<std_srvs::srv::Trigger>(
      "music_resume", std::bind(&MusicNode::resume_callback, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "Music node started");
}

MusicNode::~MusicNode() {
  this->stop_music_ = true;
  if (this->publish_thread_.joinable()) {
    this->publish_thread_.join();
  }
}

void MusicNode::publish_audio(const std::string &file_path) {

  this->is_thread_alive_ = true;

  audio_common::WaveFile wf(file_path);
  if (!wf.open()) {
    RCLCPP_ERROR(this->get_logger(), "Error opening audio file: %s",
                 file_path.c_str());
    return;
  }

  // Create rate
  std::chrono::nanoseconds period(
      (int)(1e9 * this->chunk_ / wf.get_sample_rate()));
  rclcpp::Rate pub_rate(period);
  std::vector<float> data(this->chunk_);

  while (!this->stop_music_) {
    while (wf.read(data, this->chunk_)) {

      auto msg = audio_common_msgs::msg::AudioStamped();
      msg.header.frame_id = this->frame_id_;
      msg.header.stamp = this->get_clock()->now();
      msg.audio.audio_data.float32_data = data;
      msg.audio.info.channels = wf.get_num_channels();
      msg.audio.info.chunk = this->chunk_;
      msg.audio.info.format = 1;
      msg.audio.info.rate = wf.get_sample_rate();

      this->player_pub_->publish(msg);
      pub_rate.sleep();

      if (this->pause_music_) {
        std::unique_lock<std::mutex> lock(this->pause_mutex_);
        this->pause_cv_.wait(lock, [&]() { return !this->pause_music_; });
      }

      if (this->stop_music_) {
        break;
      }
    }

    wf.rewind();
    if (!this->audio_loop_ || this->stop_music_) {
      break;
    }
  }

  this->is_thread_alive_ = false;
}

void MusicNode::play_callback(
    const std::shared_ptr<audio_common_msgs::srv::MusicPlay::Request> request,
    std::shared_ptr<audio_common_msgs::srv::MusicPlay::Response> response) {

  if (this->is_thread_alive_) {
    RCLCPP_WARN(this->get_logger(), "There is other music playing");
    response->success = false;
    return;
  }

  if (this->publish_thread_.joinable()) {
    this->publish_thread_.join();
  }

  std::string path = request->file_path;
  if (path.empty()) {
    path = ament_index_cpp::get_package_share_directory("audio_common") +
           "/samples/" + request->audio + ".wav";
  }

  if (!std::ifstream(path).good()) {
    RCLCPP_ERROR(this->get_logger(), "File %s not found", path.c_str());
    response->success = false;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Playing %s", path.c_str());
  this->audio_loop_ = request->loop;
  this->pause_music_ = false;
  this->stop_music_ = false;
  response->success = true;

  this->publish_thread_ = std::thread(&MusicNode::publish_audio, this, path);
}

void MusicNode::pause_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

  if (this->is_thread_alive_) {
    this->pause_music_ = true;
    RCLCPP_INFO(this->get_logger(), "Music paused");
    response->success = true;

  } else {
    RCLCPP_WARN(this->get_logger(), "No music to pause");
    response->success = false;
  }
}

void MusicNode::resume_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

  if (this->is_thread_alive_) {
    this->pause_music_ = false;
    this->pause_cv_.notify_all();
    RCLCPP_INFO(this->get_logger(), "Music resumed");
    response->success = true;

  } else {
    RCLCPP_WARN(this->get_logger(), "No music to resume");
    response->success = false;
  }
}

void MusicNode::stop_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

  if (this->is_thread_alive_) {
    this->stop_music_ = true;
    this->publish_thread_.join();
    RCLCPP_INFO(this->get_logger(), "Music stopped");
    response->success = true;

  } else {
    RCLCPP_WARN(this->get_logger(), "No music to stop");
    response->success = false;
  }
}
