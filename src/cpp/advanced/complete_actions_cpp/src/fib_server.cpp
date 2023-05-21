/**
 * Example Fibonacci computation action server source code.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 10, 2022
 */

#include <chrono>

#include <complete_actions_cpp/fib_server.hpp>

using namespace std::chrono_literals;

//! Status topic must have transient local durability
static const rmw_qos_profile_t status_qos_profile = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  5,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

/**
 * @brief Creates a new FibonacciComputer node.
 */
FibonacciComputer::FibonacciComputer()
: Node("fibonacci_computer")
{
  //! Little point in having a different QoS for services rather than the
  //! default, but feedbacks and statuses might be important
  server_opts_.allocator = rcl_get_default_allocator();
  server_opts_.goal_service_qos = rmw_qos_profile_services_default;
  server_opts_.cancel_service_qos = rmw_qos_profile_services_default;
  server_opts_.result_service_qos = rmw_qos_profile_services_default;
  server_opts_.status_topic_qos = status_qos_profile;
  server_opts_.feedback_topic_qos = rmw_qos_profile_default;
  server_clbk_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  //! Works will be queried for and executed every 10 seconds
  work_timer_ = this->create_wall_timer(
    10s,
    std::bind(
      &FibonacciComputer::work_activation_clbk,
      this));

  //! Create the action server by specifying the node that handles it,
  //! and the three handler routines for the three computation stages
  //! You could pass other arguments:
  //! - rcl_action_server_options_t struct to hold QoS policies and more
  //! - Callback group (i.e. thread) on which the server's back-end will run
  fib_server_ = rclcpp_action::create_server<Fibonacci>(
    this,
    "~/fibonacci",
    std::bind(
      &FibonacciComputer::handle_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &FibonacciComputer::handle_cancel,
      this,
      std::placeholders::_1),
    std::bind(
      &FibonacciComputer::handle_accepted,
      this,
      std::placeholders::_1),
    server_opts_,
    server_clbk_group_);

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Handles a new goal request.
 *
 * @param uuid ID of the new request.
 * @param goal Pointer to goal object to parse.
 * @return Handling result code.
 */
rclcpp_action::GoalResponse FibonacciComputer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  FibonacciGoalSharedPtr goal)
{
  //! This server accepts requests up to order 20
  if (goal->order > 20) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Received invalid request (%s) of order %d REJECTED",
      rclcpp_action::to_string(uuid).c_str(),
      goal->order);
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Received request (%s) of order %d ACCEPTED",
    rclcpp_action::to_string(uuid).c_str(),
    goal->order);
  //! Returning the following code will force the middleware to
  //! immediately move the goal to the executing state
  //! Here we decide when to execute each goal
  return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

/**
 * @brief Handles a goal cancellation request.
 *
 * @param goal_handle Handle to the goal object.
 * @return Cancellation operation status.
 */
rclcpp_action::CancelResponse FibonacciComputer::handle_cancel(
  const FibonacciGoalHandleSharedPtr goal_handle)
{
  //! This server doesn't cancel short computations
  if (goal_handle->get_goal()->order < 10) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Received cancellation request for goal (%s) REJECTED",
      rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
    return rclcpp_action::CancelResponse::REJECT;
  }

  new_goal_ = nullptr;

  //! Note that this will tell the middleware to only ATTEMPT cancellation
  //! since the scheme is deferred: a flag will be raised and will have to be
  //! checked by every worker routine (not handled by the middleware)
  RCLCPP_WARN(
    this->get_logger(),
    "Received cancellation request for goal (%s) ACCEPTED",
    rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief Executed when a goal request execution is started.
 *
 * @param goal_handle Handle to the goal object.
 */
void FibonacciComputer::handle_accepted(
  const FibonacciGoalHandleSharedPtr goal_handle)
{
  //! Here you could place the handle pointer in a queue to be processed later
  new_goal_ = goal_handle;
}

/**
 * @brief Computes the Fibonacci sequence up to a given order.
 *
 * @param goal_handle Handle to the goal object.
 */
void FibonacciComputer::compute(
  const FibonacciGoalHandleSharedPtr goal_handle)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Starting computation for request (%s)",
    rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());

  //! This object helps us to define sleep times (in real time)
  rclcpp::WallRate loop_rate(1s);

  //! Feedback publishing API requires a shared_ptr
  auto feedback = std::make_shared<Fibonacci::Feedback>();
  auto & sequence = feedback->partial_sequence;

  //! Same holds for the result message
  auto result = std::make_shared<Fibonacci::Result>();

  const auto goal = goal_handle->get_goal();
  int order = goal->order;

  //! This implementation refuses order 0
  if (order == 0) {
    goal_handle->abort(result); //! Terminal API to call
    RCLCPP_ERROR(
      this->get_logger(),
      "Request (%s) has invalid order ABORTED",
      rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
    return;
  }

  // Initialize sequence
  sequence.push_back(0);
  sequence.push_back(1);

  // Computation loop
  //! Always check if rclcpp::ok while computing since you need it to
  //! interact with the middleware!
  for (int i = 1; (i < order) && rclcpp::ok(); i++) {
    //! Check if there is a cancel request, and in case mark the goal as such
    if (goal_handle->is_canceling()) {
      // Publish what has been computed so far
      result->set__sequence(sequence);
      goal_handle->canceled(result); //! Terminal API to call
      RCLCPP_WARN(
        this->get_logger(),
        "Computation (%s) CANCELED",
        rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
      return;
    }

    // Update sequence
    sequence.push_back(sequence[i] + sequence[i - 1]);

    //! Publish feedback (that we're building upon)
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(
      this->get_logger(),
      "Published feedback for goal (%s)",
      rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());

    //! This simulates computational overhead
    loop_rate.sleep();
  }

  // Publish result
  //! ALWAYS check if the middelware is ok!
  if (rclcpp::ok()) {
    result->set__sequence(sequence);
    goal_handle->succeed(result); //! Terminal API to call
    RCLCPP_INFO(
      this->get_logger(),
      "Goal (%s) completed",
      rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
  }
}

/**
 * @brief Starts queued jobs.
 */
void FibonacciComputer::work_activation_clbk()
{
  if (new_goal_) {
    FibonacciGoalHandleSharedPtr goal_handle = new_goal_;
    new_goal_ = nullptr;
    RCLCPP_WARN(
      this->get_logger(),
      "Starting execution of goal (%s)",
      rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
    goal_handle->execute(); //! Update internal goal state
    std::thread{
      std::bind(
        &FibonacciComputer::compute,
        this,
        std::placeholders::_1),
      goal_handle
    }.detach();
  }
}
