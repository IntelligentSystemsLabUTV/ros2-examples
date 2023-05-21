/**
 * Fibonacci computation action client.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 10, 2022
 */

#include <sstream>

//! Service calls will all handle futures
#include <future>

//! Necessary to define pointers to addres many action-related objects
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp> //! Necessary to use actions

//! Our interfaces
#include <ros2_examples_interfaces/action/fibonacci.hpp>

//! Again, let's make our life easier
using Fibonacci = ros2_examples_interfaces::action::Fibonacci;
using FibonacciGoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;
using FibonacciGoalHandleSharedPtr = FibonacciGoalHandle::SharedPtr;
using CancelResponse = action_msgs::srv::CancelGoal::Response;
using CancelResponseSharedPtr = CancelResponse::SharedPtr;

/**
 * Sends a request to compute the Fibonacci sequence up to a given number.
 */
class FibonacciClient : public rclcpp::Node
{
public:
  FibonacciClient();
  std::string get_result_str();

  //! These are required to wrap operations on the (private) back-end
  std::shared_future<FibonacciGoalHandleSharedPtr> send_goal(int order);
  std::shared_future<FibonacciGoalHandle::WrappedResult> request_result(
    FibonacciGoalHandleSharedPtr goal_handle);
  std::shared_future<CancelResponseSharedPtr> request_cancel(
    FibonacciGoalHandleSharedPtr goal_handle);

private:
  std::stringstream result_ss_;

  rclcpp::CallbackGroup::SharedPtr client_clbk_group_;
  rcl_action_client_options_t action_client_opts_{};

  //! Client goes with options needed to send goal requests
  rclcpp_action::Client<Fibonacci>::SharedPtr client_;
  rclcpp_action::Client<Fibonacci>::SendGoalOptions client_opts_;

  //! Name of the following callbacks says it all, but pay attention to their
  //! signatures and argument types!
  void goal_response_clbk(
    FibonacciGoalHandleSharedPtr goal_handle);
  void feedback_callback(
    FibonacciGoalHandleSharedPtr goal_handle,
    const std::shared_ptr<const Fibonacci::Feedback> feedback);
  void result_callback(const FibonacciGoalHandle::WrappedResult & result);
  void cancel_callback(CancelResponseSharedPtr cancel_resp);
};
