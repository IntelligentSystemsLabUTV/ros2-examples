/**
 * Example Fibonacci computation action server.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 10, 2022
 */

#ifndef FIB_SERVER_HPP
#define FIB_SERVER_HPP

//! Required to handle all the following shared pointers
#include <memory>

//! Required to handle thread generation when accepting goals
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp> //! Required when handling actions
#include <ros2_examples_interfaces/action/fibonacci.hpp> //! Fibonacci action definition

//! Action-related types can get quite long, so let's simplify
using Fibonacci = ros2_examples_interfaces::action::Fibonacci;
using FibonacciGoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

//! Define these manually to enforce const here and in the following
using FibonacciGoalSharedPtr = std::shared_ptr<const Fibonacci::Goal>;
using FibonacciGoalHandleSharedPtr = std::shared_ptr<FibonacciGoalHandle>;

//! Pay attention to the types hell...

/**
 * Action server that computes the Fibonacci sequence up to a point.
 */
class FibonacciComputer : public rclcpp::Node
{
public:
  FibonacciComputer();

private:
  //! Action server (like for services)
  rclcpp_action::Server<Fibonacci>::SharedPtr fib_server_;

  rclcpp::CallbackGroup::SharedPtr server_clbk_group_;
  rcl_action_server_options_t server_opts_{};
  rclcpp::TimerBase::SharedPtr work_timer_;
  void work_activation_clbk();
  FibonacciGoalHandleSharedPtr new_goal_ = nullptr;

  //! Following 3 routines are required by the API, first 2 return from enums
  //! Goal request handler routine
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    FibonacciGoalSharedPtr goal);

  //! Goal cancellation request handler routine
  rclcpp_action::CancelResponse handle_cancel(const FibonacciGoalHandleSharedPtr goal_handle);

  //! Goal acceptance handler routine
  void handle_accepted(const FibonacciGoalHandleSharedPtr goal_handle);

  //! Computation routine
  void compute(const FibonacciGoalHandleSharedPtr goal_handle);
};

#endif
