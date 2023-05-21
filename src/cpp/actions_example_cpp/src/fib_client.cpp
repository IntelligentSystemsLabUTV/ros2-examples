/**
 * Fibonacci computation action client source code.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 10, 2022
 */

#include <chrono>
#include <sstream>
#include <stdexcept>

#include <actions_example_cpp/fib_client.hpp>

#define UNUSED(arg) (void)(arg)

/**
 * @brief Creates a new FibonacciClient node.
 */
FibonacciClient::FibonacciClient()
: Node("fibonacci_client")
{
  //! Create the action client like a service client
  client_ = rclcpp_action::create_client<Fibonacci>(
    this,
    "/fibonacci_computer/fibonacci");

  //! Initialize client options: bind all callbacks
  client_opts_.goal_response_callback = std::bind(
    &FibonacciClient::goal_response_clbk,
    this,
    std::placeholders::_1);
  client_opts_.feedback_callback = std::bind(
    &FibonacciClient::feedback_callback,
    this,
    std::placeholders::_1,
    std::placeholders::_2);
  client_opts_.result_callback = std::bind(
    &FibonacciClient::result_callback,
    this,
    std::placeholders::_1);

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Called when the goal is sent to the server over the goal service.
 *
 * @param future_resp Goal handle future.
 */
void FibonacciClient::goal_response_clbk(
  FibonacciGoalHandleSharedPtr goal_handle)
{
  //! Check if the handle is valid, i.e. the goal was accepted or rejected
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Goal was accepted by server (%s at %f)",
      rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(),
      goal_handle->get_goal_stamp().seconds());
  }
}

/**
 * @brief Prints the feedback message.
 *
 * @param goal_handle Handle to the goal being processed.
 * @param feedback New feedback message.
 */
void FibonacciClient::feedback_callback(
  FibonacciGoalHandleSharedPtr goal_handle,
  const std::shared_ptr<const Fibonacci::Feedback> feedback)
{
  UNUSED(goal_handle);
  // Just print the partial sequence we just got
  std::stringstream ss("");
  ss << "Partial sequence: ";
  for (auto number : feedback->partial_sequence) {
    ss << number << " ";
  }
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}

/**
 * @brief Handles the goal cancellation response.
 *
 * @param cancel_resp Cancellation response message.
 */
void FibonacciClient::cancel_callback(CancelResponseSharedPtr cancel_resp)
{
  switch (cancel_resp->return_code) {
    case CancelResponse::ERROR_NONE:
      RCLCPP_WARN(this->get_logger(), "Goal CANCELED");
      break;
    case CancelResponse::ERROR_REJECTED:
      RCLCPP_ERROR(this->get_logger(), "Goal cancellation request REJECTED");
      break;
    case CancelResponse::ERROR_UNKNOWN_GOAL_ID:
      RCLCPP_ERROR(this->get_logger(), "Unkown goal ID");
      break;
    case CancelResponse::ERROR_GOAL_TERMINATED:
      RCLCPP_WARN(this->get_logger(), "Goal TERMINATED");
      break;
    default:
      break;
  }
}

/**
 * @brief Called when the result server just sent the result.
 *
 * @param result Wrapped result object.
 */
void FibonacciClient::result_callback(
  const FibonacciGoalHandle::WrappedResult & result)
{
  //! This is called when the server has just sent the result back!
  //! The result object holds:
  //! - Code indicating whether the result is successful, canceled or aborted
  //! - Result as specified by the interface
  //! This routine is a good starting template for this kind of callback

  RCLCPP_INFO(
    this->get_logger(),
    "Got result of (%s)",
    rclcpp_action::to_string(result.goal_id).c_str());

  // Parse the result
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "ABORTED");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "CANCELED");
      result_ss_ << "(Partial) ";
      break;
    case rclcpp_action::ResultCode::UNKNOWN:
      RCLCPP_ERROR(this->get_logger(), "UNKNOWN");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  // Parse the (successful) result
  result_ss_ << "Result: ";
  for (auto number : result.result->sequence) {
    result_ss_ << number << " ";
  }
}

/**
 * @brief Getter for result char string pointer.
 *
 * @return Result char string pointer.
 */
std::string FibonacciClient::get_result_str()
{
  return result_ss_.str();
}

/**
 * @brief Sends a new goal to the server.
 *
 * @param order Order up to which the Fibonacci sequence has to be computed.
 * @return Goal handle future.
 */
std::shared_future<FibonacciGoalHandleSharedPtr> FibonacciClient::send_goal(
  int order)
{
  //! Note that since this whole system relies entirely on callbacks, this
  //! just has to send the goal, everything else will happen automatically
  //! afterwards!
  //! I.e. actions induce event-based asynchronous programming, which you can
  //! make synchronous by waiting explicitly on futures and similar objects

  // Initialize result
  result_ss_.str("");

  //! Initialize goal request object
  Fibonacci::Goal goal_request{};
  goal_request.set__order(order);

  //! Wait for the server to become available
  while (!client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Server not available...");
    if (!rclcpp::ok()) {
      throw std::runtime_error("Middleware crashed");
    }
  }

  //! Send goal to goal server, setting callbacks for this request, and
  //! return the related future
  return client_->async_send_goal(goal_request, client_opts_);
}

/**
 * @brief Requests results for a given goal.
 *
 * @param goal_handle Goal handle to request result for.
 * @return Goal WrappedResult object future.
 */
std::shared_future<FibonacciGoalHandle::WrappedResult> FibonacciClient::request_result(
  FibonacciGoalHandleSharedPtr goal_handle)
{
  return client_->async_get_result(
    goal_handle,
    std::bind(
      &FibonacciClient::result_callback,
      this,
      std::placeholders::_1));
}

/**
 * @brief Requests cancellation of a given goal.
 *
 * @param goal_handle Goal handle to request cancellation of.
 * @return Cancellation response object future.
 */
std::shared_future<CancelResponseSharedPtr> FibonacciClient::request_cancel(
  FibonacciGoalHandleSharedPtr goal_handle)
{
  return client_->async_cancel_goal(
    goal_handle,
    std::bind(
      &FibonacciClient::cancel_callback,
      this,
      std::placeholders::_1));
}
