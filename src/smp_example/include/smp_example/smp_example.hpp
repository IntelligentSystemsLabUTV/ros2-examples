/**
 * Definitions of nodes for this example.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * November 26, 2021
 */

#ifndef SMP_EXAMPLE_HPP
#define SMP_EXAMPLE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>

/**
 * Simple node with two subscribers, which callbacks will be executed on
 * different threads.
 */
class SMPNode : public rclcpp::Node
{
public:
  SMPNode();

private:
  //! The main difference is the necessity to create callback groups,
  //! intended as separate queues from which call
  rclcpp::CallbackGroup::SharedPtr clbk_group_1_;
  rclcpp::CallbackGroup::SharedPtr clbk_group_2_;

  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub_1_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub_2_;

  void sub_1_clbk(const std_msgs::msg::UInt64::SharedPtr msg);
  void sub_2_clbk(const std_msgs::msg::UInt64::SharedPtr msg);
};

/**
 * Publisher node for this example.
 */
class PubNode : public rclcpp::Node
{
public:
  PubNode(unsigned int period);

private:
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr count_pub_;

  rclcpp::TimerBase::SharedPtr pub_timer_;

  void timer_clbk_(void);

  unsigned long int count_;
};

#endif
