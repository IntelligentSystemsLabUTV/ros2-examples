/**
 * Chatter nodes that use intra-process communcation allowed by ROS 2.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 22, 2022
 */

#ifndef INTRAPC_CHATTERS_HPP
#define INTRAPC_CHATTERS_HPP

#include <chrono>
#include <cinttypes>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

using namespace std_msgs::msg;

/**
 * Talker node.
 */
class IPCTalker : public rclcpp::Node
{
public:
  //! The NodeOptions must explicitly specify the use of Intra-PC
  IPCTalker(const std::string & name = "ipc_talker")
  : rclcpp::Node(name, rclcpp::NodeOptions().use_intra_process_comms(true)),
    msg_count_(1)
  {
    // Initialize publisher
    message_pub_ = this->create_publisher<String>(
      "/examples/test_topic",
      rclcpp::QoS(10)
    );

    // Initialize publishing timer
    pub_timer_ = this->create_wall_timer(
      500ms,
      [this]() -> void {
        //! Initialize the message by getting a unique_ptr to it
        String::UniquePtr new_msg(new String{});
        new_msg->set__data("Hello " + std::to_string(msg_count_));
        RCLCPP_INFO(
          this->get_logger(),
          "Publishing message %d (0x%" PRIXPTR ")",
          msg_count_,
          reinterpret_cast<std::uintptr_t>(new_msg.get())
        );
        //! Publish it by marking the object for explicit transfer
        message_pub_->publish(std::move(new_msg));
        msg_count_++;
      }
    );

    RCLCPP_INFO(this->get_logger(), "Node initialized");
  }

private:
  rclcpp::Publisher<String>::SharedPtr message_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  uint32_t msg_count_;
};

/**
 * Listener node.
 */
class IPCListener : public rclcpp::Node
{
public:
  //! As above
  IPCListener(const std::string & name = "ipc_listener")
  : rclcpp::Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Initialize subscription
    message_sub_ = this->create_subscription<String>(
      "/examples/test_topic",
      rclcpp::QoS(10),
      //! Callback now takes a UniquePtr to the message
      [this](String::UniquePtr new_msg) -> void {
        RCLCPP_INFO(
          this->get_logger(),
          "%s (0x%" PRIXPTR ")",
          new_msg->data.c_str(),
          reinterpret_cast<std::uintptr_t>(new_msg.get())
        );
      }
    );

    RCLCPP_INFO(this->get_logger(), "Node initialized");
  }

private:
  rclcpp::Subscription<String>::SharedPtr message_sub_;
};

#endif
