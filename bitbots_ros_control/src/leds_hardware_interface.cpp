#include <bitbots_ros_control/leds_hardware_interface.h>
#include <bitbots_ros_control/utils.h>

namespace bitbots_ros_control {
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

LedsHardwareInterface::LedsHardwareInterface(rclcpp::Node::SharedPtr nh, std::shared_ptr<DynamixelDriver> &driver,
                                             uint8_t id, uint8_t num_leds, uint8_t start_number) {
  nh_ = nh;
  driver_ = driver;
  id_ = id;
  leds_.resize(num_leds);
  start_number_ = start_number;
  // we want to write the LEDs in the beginning to show that ros control started successfully. set LED 1 white
  write_leds_ = true;
  leds_[0] = std_msgs::msg::ColorRGBA();
  leds_[0].r = 0.3;
  leds_[0].g = 0.3;
  leds_[0].b = 0.3;
  leds_[0].a = 1.0;
}

bool LedsHardwareInterface::init() {
  if (start_number_ == 0) {
    leds_service_ = nh_->create_service<bitbots_msgs::srv::Leds>(
        "/set_leds", std::bind(&LedsHardwareInterface::setLeds, this, _1, _2));
  }
  sub0_ = nh_->create_subscription<std_msgs::msg::ColorRGBA>("/led" + std::to_string(start_number_), 1,
                                                             std::bind(&LedsHardwareInterface::ledCb0, this, _1));
  sub1_ = nh_->create_subscription<std_msgs::msg::ColorRGBA>("/led" + std::to_string(start_number_ + 1), 1,
                                                             std::bind(&LedsHardwareInterface::ledCb1, this, _1));
  sub2_ = nh_->create_subscription<std_msgs::msg::ColorRGBA>("/led" + std::to_string(start_number_ + 2), 1,
                                                             std::bind(&LedsHardwareInterface::ledCb2, this, _1));
  return true;
}

// todo this could be done more clever and for a general number of leds
void LedsHardwareInterface::ledCb0(std_msgs::msg::ColorRGBA msg) {
  // only write to bus if there is actually a change
  if (msg.r != leds_[0].r || msg.g != leds_[0].g || msg.b != leds_[0].b) {
    leds_[0] = msg;
    write_leds_ = true;
  }
}

void LedsHardwareInterface::ledCb1(std_msgs::msg::ColorRGBA msg) {
  // only write to bus if there is actually a change
  if (msg.r != leds_[1].r || msg.g != leds_[1].g || msg.b != leds_[1].b) {
    leds_[1] = msg;
    write_leds_ = true;
  }
}

void LedsHardwareInterface::ledCb2(std_msgs::msg::ColorRGBA msg) {
  // only write to bus if there is actually a change
  if (msg.r != leds_[2].r || msg.g != leds_[2].g || msg.b != leds_[2].b) {
    leds_[2] = msg;
    write_leds_ = true;
  }
}

void LedsHardwareInterface::read(const rclcpp::Time &t, const rclcpp::Duration &dt) {}

void LedsHardwareInterface::setLeds(const std::shared_ptr<bitbots_msgs::srv::Leds::Request> req,
                                    std::shared_ptr<bitbots_msgs::srv::Leds::Response> resp) {
  RCLCPP_WARN_STREAM(nh_->get_logger(), "service");
  if (req->leds.size() != leds_.size()) {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "You are trying to set " << req->leds.size() << " leds while the board has "
                                                                   << leds_.size() << " leds.");
  }

  for (size_t i = 0; i < leds_.size(); i++) {
    // return current state of LEDs
    resp->previous_leds.push_back(leds_[i]);

    if (req->leds[i].r > 1.0f || req->leds[i].r < 0.0f || req->leds[i].g > 1.0f || req->leds[i].g < 0.0f ||
        req->leds[i].b > 1.0f || req->leds[i].b < 0.0f) {
      RCLCPP_WARN_STREAM(nh_->get_logger(), "You tried to set LED_" << i << " to a value not between 0 and 1");
    }
    leds_[i] = req->leds[i];
  }
  write_leds_ = true;
}

uint32_t rgba_to_int32(std_msgs::msg::ColorRGBA rgba) {
  uint32_t led = ((uint8_t)(rgba.r * 255));
  led |= ((uint8_t)(rgba.g * 255)) << 8;
  led |= ((uint8_t)(rgba.b * 255)) << 16;
  led |= ((uint8_t)(rgba.a * 255)) << 24;
  return led;
}

void LedsHardwareInterface::write(const rclcpp::Time &t, const rclcpp::Duration &dt) {
  if (write_leds_) {
    // resort LEDs to go from left to right
    driver_->writeRegister(id_, "LED_2", rgba_to_int32(leds_[0]));
    driver_->writeRegister(id_, "LED_1", rgba_to_int32(leds_[1]));
    driver_->writeRegister(id_, "LED_0", rgba_to_int32(leds_[2]));

    write_leds_ = false;
  }
}
}  // namespace bitbots_ros_control