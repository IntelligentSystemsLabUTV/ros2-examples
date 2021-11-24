/**
 * Multithreaded node definition.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * November 24, 2021
 */

#ifndef SMP_NODE_HPP
#define SMP_NODE_HPP

#include <rclcpp/rclcpp.hpp>

/**
 * Simple node with two timers, which callbacks will be executed on
 * different threads.
 */
class SMPNode : public rclcpp::Node
{
public:
  SMPNode(unsigned int T1, unsigned int T2);

private:
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;

  //! The main difference is the necessity to create callback groups,
  //! intended as separate queues from which call
  rclcpp::CallbackGroup::SharedPtr clbk_group_1_;
  rclcpp::CallbackGroup::SharedPtr clbk_group_2_;

  void timer_clbk_1(void);
  void timer_clbk_2(void);

  // Used in log prints
  unsigned long int count_1_;
  unsigned long int count_2_;
};

#endif
