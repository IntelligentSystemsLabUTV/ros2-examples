/**
 * Dynamic subscriber example.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 15, 2022
 */

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std_msgs::msg;
using namespace std::chrono_literals;

/**
 * Subscriber node that dynamically resubscribes to a given topic.
 * Run this alongside this package's publisher.
 */
//! For the sake of simplicity we'll write everything in here this time
class ResettingSub : public rclcpp::Node
{
public:
  ResettingSub()
  : Node("resetting_subscriber")
  {
    //! We just have to initialize the timer
    sub_timer_ = this->create_wall_timer(
      5s,
      [this]() -> void {
        //! This just has to toggle topic subscription
        if (sub_) { //! Operator checks if it is nullptr
          RCLCPP_WARN(this->get_logger(), "De-subscribing from topic");
          //! Resetting the shared_ptr will make the Subscription fall out of
          //! scope, thus being destroyed
          //! ref.: https://answers.ros.org/question/354792/rclcpp-how-to-unsubscribe-from-a-topic/
          sub_.reset();
        } else {
          RCLCPP_WARN(this->get_logger(), "Re-subscribing to topic");
          //! We just create a new Subscription
          sub_ = this->create_subscription<String>(
            "/examples/test_topic",
            rclcpp::QoS(10),
            [this](String::SharedPtr msg) -> void {
              RCLCPP_INFO(this->get_logger(), msg->data.c_str());
            });
          //! And that's it. The same could be done with a Publisher,
          //! but being it asynchronous by default it's usually less important
        }
      });

    RCLCPP_INFO(this->get_logger(), "Node initialized");
  }

private:
  //! Note that this, apart from the type wrapping, is a classic shared_ptr
  rclcpp::Subscription<String>::SharedPtr sub_;

  rclcpp::TimerBase::SharedPtr sub_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto sub_node = std::make_shared<ResettingSub>();
  rclcpp::spin(sub_node);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
