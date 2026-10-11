#pragma once

#include <chrono>
#include <functional>
#include <optional>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

/// Fetching the field dimensions from the parameter blackboard.
namespace bitbots_head_mover {

/// The field size as the parameter blackboard reports it.
struct FieldDimensions {
  double length = 0.0;
  double width = 0.0;
};

/// Asks another node for the field dimensions until it answers.
///
/// The dimensions are not part of this node's parameters, they live on the
/// parameter blackboard. That node may come up after this one, so a single
/// attempt at startup is not enough. Every request is asynchronous and its
/// response is handled by whichever executor spins the node, so nothing ever
/// blocks that executor or spins the node on the side. A request that has not
/// been answered by the next attempt is dropped and sent again, so a request
/// lost on the way does not stall the retries.
class FieldDimensionClient {
 public:
  using Callback = std::function<void(const FieldDimensions&)>;

  /// Starts requesting the dimensions right away.
  ///
  /// on_received is called once, from the executor spinning the node, with
  /// the first usable dimensions.
  FieldDimensionClient(rclcpp::Node::SharedPtr node, const std::string& remote_node,
                       std::chrono::nanoseconds retry_period, Callback on_received);

  /// Whether usable dimensions were received.
  bool received() const { return received_; }

 private:
  using GetParameters = rcl_interfaces::srv::GetParameters;

  /// Send a request, replacing one that is still unanswered.
  void attempt();
  /// Check and adopt an answer from the remote node.
  void handleResponse(rclcpp::Client<GetParameters>::SharedFuture future);

  rclcpp::Node::SharedPtr node_;
  std::string remote_node_;
  Callback on_received_;
  rclcpp::Client<GetParameters>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  std::optional<int64_t> pending_request_;
  bool received_ = false;
};

}  // namespace bitbots_head_mover
