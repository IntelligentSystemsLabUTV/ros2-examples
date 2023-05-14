/**
 * Fibonacci computation action client main source file.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 10, 2022
 */

#include <iostream>
#include <cstdlib>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "../include/actions_example/fib_client.hpp"

/**
 * @brief Wraps a goal cancellation request routine.
 *
 * @param node_ptr Client node pointer.
 * @param goal_handle Handle to the goal to cancel.
 */
void cancel_goal(
  std::shared_ptr<FibonacciClient> node_ptr,
  FibonacciGoalHandleSharedPtr goal_handle)
{
  // Send the goal cancellation request
  std::shared_future<CancelResponseSharedPtr> cancel_resp =
    node_ptr->request_cancel(goal_handle);

  // Wait for and parse the result
  //! As for this process' semantics, we don't need to do much more than
  //! print what we got back
#ifndef ADVANCED
  switch (rclcpp::spin_until_future_complete(node_ptr, cancel_resp)) {
#else
  rclcpp::executors::MultiThreadedExecutor smp_executor;
  switch (rclcpp::executors::spin_node_until_future_complete(
      smp_executor,
      node_ptr,
      cancel_resp))
  {
#endif
    case rclcpp::FutureReturnCode::SUCCESS:
      break;
    //! Other codes aren't important in this context
    default:
      std::cerr << "Error while waiting for cancellation reponse" << std::endl;
      return;
  }
  //! The callback will do the rest of the job
}

int main(int argc, char ** argv)
{
  // Parse input arguments
  if (argc < 2) {
    std::cerr << "Usage:\n\tfib_client ORDER" << std::endl;
    exit(EXIT_FAILURE);
  }
  int order = atoi(argv[1]);

  // Initialize ROS 2 back-end
  rclcpp::init(argc, argv);
  auto client_node = std::make_shared<FibonacciClient>();

  //! From async_send_goal documentation:
  //! If the goal is accepted by an action server, the returned future is set
  //! to a ClientGoalHandle. If the goal is rejected by an action server, then
  //! the future is set to a nullptr.
  //! The goal handle is used to monitor the status of the goal and get the
  //! final result.

  //! Now just like for service clients, we need to spin the node while checking
  //! for the future's value
  //! Thankfully, these template library functions still work, and the following
  //! code can be used as a template
  //! This has to be done twice: to wait for the goal request and for the result

  // Send goal request and wait for the goal request to get an answer
  //! This starts the event-based callback scheme
  std::shared_future<FibonacciGoalHandleSharedPtr> goal_resp =
    client_node->send_goal(order);
#ifndef ADVANCED
  switch (rclcpp::spin_until_future_complete(client_node, goal_resp)) {
#else
  rclcpp::executors::MultiThreadedExecutor smp_executor;
  switch (rclcpp::executors::spin_node_until_future_complete(
      smp_executor,
      client_node,
      goal_resp))
  {
#endif
    case rclcpp::FutureReturnCode::SUCCESS:
      break;
    case rclcpp::FutureReturnCode::INTERRUPTED:
      std::cerr << "Interrupted while sending goal" << std::endl;
      rclcpp::shutdown();
      exit(EXIT_FAILURE);
    default:
      std::cerr << "Unknown error with the handle future" << std::endl;
      rclcpp::shutdown();
      exit(EXIT_FAILURE);
  }
  //! Check if the goal has been rejected
  if (!goal_resp.get()) {
    rclcpp::shutdown();
    exit(EXIT_FAILURE);
  }
  FibonacciGoalHandleSharedPtr goal_handle = goal_resp.get();

  // For the sake of this test, if the order is 20 the goal is canceled
  if (order == 20) {
    std::cerr << "Order too high, requesting goal cancellation" << std::endl;
    cancel_goal(client_node, goal_handle);
    rclcpp::shutdown();
    exit(EXIT_SUCCESS);
  }

  // Wait for the result to be sent back
  std::shared_future<FibonacciGoalHandle::WrappedResult> result_resp =
    client_node->request_result(goal_handle);
#ifndef ADVANCED
  switch (rclcpp::spin_until_future_complete(client_node, result_resp)) {
#else
  switch (rclcpp::executors::spin_node_until_future_complete(
      smp_executor,
      client_node,
      result_resp))
  {
#endif
    case rclcpp::FutureReturnCode::SUCCESS:
      break;
    case rclcpp::FutureReturnCode::INTERRUPTED:
      std::cerr << "Interrupted while waiting for result" << std::endl;
      rclcpp::shutdown();
      exit(EXIT_FAILURE);
    default:
      std::cerr << "Unknown error with the result future" << std::endl;
      rclcpp::shutdown();
      exit(EXIT_FAILURE);
  }

  // Print the result
  //! Note that we could parse the WrappedResult here, but the callback will
  //! do so for us
  std::string result_string = client_node->get_result_str();
  if (result_string.length()) {
    //! String would be empty if this was aborted
    std::cout << result_string << std::endl;
  }
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
