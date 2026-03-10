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

#ifndef AUDIO_COMMON__MUSIC_NODE
#define AUDIO_COMMON__MUSIC_NODE

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <audio_common_msgs/msg/audio_stamped.hpp>
#include <audio_common_msgs/srv/music_play.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace audio_common {

class MusicNode : public rclcpp::Node {
public:
  MusicNode();
  ~MusicNode() override;

private:
  // ROS 2 parameters
  int chunk_;
  std::string frame_id_;

  // Audio control flags
  std::atomic<bool> pause_music_;
  std::atomic<bool> stop_music_;
  bool audio_loop_;
  std::atomic<bool> is_thread_alive_;

  // Threads and synchronization
  std::thread publish_thread_;
  std::mutex pause_mutex_;
  std::condition_variable pause_cv_;

  // ROS 2 publisher and services
  rclcpp::Publisher<audio_common_msgs::msg::AudioStamped>::SharedPtr
      player_pub_;
  rclcpp::Service<audio_common_msgs::srv::MusicPlay>::SharedPtr play_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;

  // Methods
  void publish_audio(const std::string &file_path);
  void play_callback(
      const std::shared_ptr<audio_common_msgs::srv::MusicPlay::Request> request,
      std::shared_ptr<audio_common_msgs::srv::MusicPlay::Response> response);
  void
  pause_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void resume_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void
  stop_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

} // namespace audio_common

#endif