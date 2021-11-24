/**
 * Multithreaded node code.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * November 24, 2021
 */

#include <chrono>
#include <thread>

#include "../include/smp_example/smp_node.hpp"

/**
 * @brief Creates an SMPNode.
 *
 * @param T1 Timer 1 period.
 * @param T2 Timer 2 period.
 */
SMPNode::SMPNode(unsigned int T1, unsigned int T2)
: Node("smp_node")
{
  //! Callback groups (i.e. pointers to-) must be initialized in this way
  //! They can be mutually exclusive or reentrant
  clbk_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  clbk_group_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  //! When creating timers, the callback group must be specified
  //! This is also true for services, whereas in topic subscriptions an
  //! appropriate object named "subscription option" must be populated and then
  //! passed to the API, like this:
  //!
  //! auto SUB_OPTS_OBJ = rclcpp::SubscriptionOptions();
  //! SUB_OPTS_OBJ.callback_group = CLBK_GRP_OBJ;
  timer1_ = this->create_wall_timer(
    std::chrono::milliseconds(T1),
    std::bind(
      &SMPNode::timer_clbk_1,
      this
    ),
    clbk_group_1_
  );
  timer2_ = this->create_wall_timer(
    std::chrono::milliseconds(T2),
    std::bind(
      &SMPNode::timer_clbk_2,
      this
    ),
    clbk_group_2_
  );

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

//! Notice below the usage of the RCLCPP_*_STREAM macros, to directly
//! access the underlying logging output stream.
/**
 * @brief First timer callback.
 */
void SMPNode::timer_clbk_1(void)
{
  count_1_++;
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Hello " << count_1_ << " from thread " << std::this_thread::get_id() << " (1)"
  );
}

/**
 * @brief Second timer callback.
 */
void SMPNode::timer_clbk_2(void)
{
  count_2_++;
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Hello " << count_2_ << " from thread " << std::this_thread::get_id() << " (2)"
  );
}
