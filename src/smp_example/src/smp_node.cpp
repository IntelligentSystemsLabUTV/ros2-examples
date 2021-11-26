/**
 * Multithreaded node code.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * November 26, 2021
 */

#include <chrono>
#include <thread>

#include "../include/smp_example/smp_example.hpp"

/**
 * @brief Creates an SMPNode.
 */
SMPNode::SMPNode()
: Node("smp_node")
{
  //! Callback groups (i.e. pointers to-) must be initialized in this way
  //! They can be mutually exclusive or reentrant
  clbk_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  clbk_group_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  //! When creating timers, the callback group must be passed to the API
  //! This is also true for services, whereas in topic subscriptions an
  //! appropriate object named "subscription option" must be populated and then
  //! passed to the API, like this:
  //!
  //! auto SUB_OPTS_OBJ = rclcpp::SubscriptionOptions();
  //! SUB_OPTS_OBJ.callback_group = CLBK_GRP_OBJ;
  //!
  //! and this is exactly our case
  rclcpp::SubscriptionOptions sub_opts_1, sub_opts_2;
  sub_opts_1.callback_group = clbk_group_1_;
  sub_opts_2.callback_group = clbk_group_2_;

  sub_1_ = this->create_subscription<std_msgs::msg::UInt64>(
    "/ros2_examples/smp_topic",
    rclcpp::QoS(10),
    std::bind(
      &SMPNode::sub_1_clbk,
      this,
      std::placeholders::_1
    ),
    sub_opts_1
  );
  sub_2_ = this->create_subscription<std_msgs::msg::UInt64>(
    "/ros2_examples/smp_topic",
    rclcpp::QoS(10),
    std::bind(
      &SMPNode::sub_2_clbk,
      this,
      std::placeholders::_1
    ),
    sub_opts_2
  );

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

//! Notice below the usage of the RCLCPP_*_STREAM macros, to directly
//! access the underlying logging output stream.
/**
 * @brief First subscriber callback.
 *
 * @param msg Message received.
 */
void SMPNode::sub_1_clbk(const std_msgs::msg::UInt64::SharedPtr msg)
{
  RCLCPP_WARN_STREAM(
    this->get_logger(),
    "Hello " << msg->data << " from thread " << std::this_thread::get_id() << " (1)"
  );
}

/**
 * @brief Second subscriber callback.
 *
 * @param msg Message received.
 */
void SMPNode::sub_2_clbk(const std_msgs::msg::UInt64::SharedPtr msg)
{
  RCLCPP_WARN_STREAM(
    this->get_logger(),
    "Hello " << msg->data << " from thread " << std::this_thread::get_id() << " (2)"
  );
}
