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

#include <chrono>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "audio_common/tts_node.hpp"
#include "audio_common/wave_file.hpp"
#include "audio_common_msgs/action/tts.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"

using namespace audio_common;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

TtsNode::TtsNode() : Node("tts_node") {

  this->declare_parameter("chunk", 4096);
  this->declare_parameter("frame_id", "");

  this->chunk_ = this->get_parameter("chunk").as_int();
  this->frame_id_ = this->get_parameter("frame_id").as_string();

  this->player_pub_ =
      this->create_publisher<audio_common_msgs::msg::AudioStamped>(
          "audio", rclcpp::SensorDataQoS());

  // Action server
  this->action_server_ = rclcpp_action::create_server<TTS>(
      this, "say", std::bind(&TtsNode::handle_goal, this, _1, _2),
      std::bind(&TtsNode::handle_cancel, this, _1),
      std::bind(&TtsNode::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "TTS node started");
}

rclcpp_action::GoalResponse
TtsNode::handle_goal(const rclcpp_action::GoalUUID &uuid,
                     std::shared_ptr<const TTS::Goal> goal) {
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
TtsNode::handle_cancel(const std::shared_ptr<GoalHandleTTS> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Canceling TTS...");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TtsNode::handle_accepted(
    const std::shared_ptr<GoalHandleTTS> goal_handle) {
  std::unique_lock<std::mutex> lock(this->goal_lock_);
  if (this->goal_handle_ != nullptr && this->goal_handle_->is_active()) {
    auto result = std::make_shared<TTS::Result>();
    this->goal_handle_->abort(result);
    this->goal_handle_ = goal_handle;
  }

  std::thread{std::bind(&TtsNode::execute_callback, this, _1), goal_handle}
      .detach();
}

void TtsNode::execute_callback(
    const std::shared_ptr<GoalHandleTTS> goal_handle) {
  auto result = std::make_shared<TTS::Result>();
  const auto goal = goal_handle->get_goal();
  std::string text = goal->text;
  std::string language = goal->language;
  int rate = static_cast<int>(goal->rate * 175);
  int volume = static_cast<int>(goal->volume * 100);

  // Create audio file using espeak
  char temp_file[] = "/tmp/tts_audio.wav";
  std::stringstream cmd;
  cmd << "espeak -v" << language << " -s" << rate << " -a" << volume << " -w "
      << temp_file << " '" << text << "'";

  int ret = std::system(cmd.str().c_str());
  if (ret != 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "espeak command failed with return code: %d", ret);
    goal_handle->abort(result);
    return;
  }

  // Read audio file
  audio_common::WaveFile wf(temp_file);
  if (!wf.open()) {
    RCLCPP_ERROR(this->get_logger(), "Error opening audio file: %s", temp_file);
    goal_handle->abort(result);
    return;
  }

  // Create rate
  std::chrono::nanoseconds period(
      (int)(1e9 * this->chunk_ / wf.get_sample_rate()));
  rclcpp::Rate pub_rate(period);
  std::vector<float> data(this->chunk_);

  // Initialize the audio message
  audio_common_msgs::msg::AudioStamped msg;
  msg.header.frame_id = this->frame_id_;

  // Publish the audio data in chunks
  while (wf.read(data, this->chunk_)) {
    if (!goal_handle->is_active()) {
      return;
    }

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }

    auto msg = audio_common_msgs::msg::AudioStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.audio.audio_data.float32_data = data;
    msg.audio.info.channels = wf.get_num_channels();
    msg.audio.info.chunk = this->chunk_;
    msg.audio.info.format = 1;
    msg.audio.info.rate = wf.get_sample_rate();

    auto feedback = std::make_shared<TTS::Feedback>();
    feedback->audio = msg;

    this->player_pub_->publish(msg);
    goal_handle->publish_feedback(feedback);
    pub_rate.sleep();
  }

  // Cleanup and set result
  std::remove(temp_file);

  result->text = text;
  goal_handle->succeed(result);
}
