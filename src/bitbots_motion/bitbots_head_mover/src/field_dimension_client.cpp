#include <bitbots_head_mover/field_dimension_client.hpp>
#include <utility>

namespace bitbots_head_mover {

namespace {
const char* const kLengthParameter = "field.size.x";
const char* const kWidthParameter = "field.size.y";
}  // namespace

FieldDimensionClient::FieldDimensionClient(rclcpp::Node::SharedPtr node, const std::string& remote_node,
                                           std::chrono::nanoseconds retry_period, Callback on_received)
    : node_(std::move(node)), remote_node_(remote_node), on_received_(std::move(on_received)) {
  client_ = node_->create_client<GetParameters>(remote_node_ + "/get_parameters");
  // Waiting for another process to come up is a matter of wall time, so the
  // retries keep going even while a simulation clock is paused
  retry_timer_ = node_->create_wall_timer(retry_period, [this] { attempt(); });
  attempt();
}

void FieldDimensionClient::attempt() {
  if (received_) {
    retry_timer_->cancel();
    return;
  }

  // A request that is still pending by now was lost or ignored. Dropping it
  // keeps the client from collecting requests that will never be answered, and
  // a late answer to it is not delivered anymore.
  if (pending_request_) {
    client_->remove_pending_request(*pending_request_);
    pending_request_.reset();
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                         "'%s' did not answer the request for the field dimensions, asking again",
                         remote_node_.c_str());
  }

  if (!client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                         "Waiting for '%s' to provide the field dimensions", remote_node_.c_str());
    return;
  }

  auto request = std::make_shared<GetParameters::Request>();
  request->names = {kLengthParameter, kWidthParameter};
  pending_request_ =
      client_
          ->async_send_request(request,
                               [this](rclcpp::Client<GetParameters>::SharedFuture future) { handleResponse(future); })
          .request_id;
}

void FieldDimensionClient::handleResponse(rclcpp::Client<GetParameters>::SharedFuture future) {
  pending_request_.reset();
  if (received_) {
    return;
  }

  const auto response = future.get();
  // The parameter service answers without any values when a requested
  // parameter is not declared
  if (response->values.size() != 2) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000, "'%s' does not provide %s and %s",
                          remote_node_.c_str(), kLengthParameter, kWidthParameter);
    return;
  }

  const auto length = rclcpp::ParameterValue(response->values[0]);
  const auto width = rclcpp::ParameterValue(response->values[1]);
  if (length.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
      width.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                          "'%s' does not provide %s and %s as doubles (got %s and %s)", remote_node_.c_str(),
                          kLengthParameter, kWidthParameter, rclcpp::to_string(length.get_type()).c_str(),
                          rclcpp::to_string(width.get_type()).c_str());
    return;
  }

  FieldDimensions dimensions{length.get<double>(), width.get<double>()};
  // A degenerate field would build an empty or nonsensical coverage map, which
  // is worth saying out loud rather than quietly steering the head
  if (!(dimensions.length > 0.0) || !(dimensions.width > 0.0)) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                          "'%s' reports a field of %.2f x %.2f m, which is not usable", remote_node_.c_str(),
                          dimensions.length, dimensions.width);
    return;
  }

  received_ = true;
  retry_timer_->cancel();
  on_received_(dimensions);
}

}  // namespace bitbots_head_mover
