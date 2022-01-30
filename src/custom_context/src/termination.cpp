/**
 * ROS 2-compliant signal handling scheme example.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 17, 2022
 */

#define MODULE_NAME "termination"

#include <iostream>
#include <unistd.h>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>

#include <ros2_examples_headers/signal_handler/signal_handler.hpp>

#include <custom_context/listener_node.hpp>

#define UNUSED(arg) (void)(arg)

using namespace ROS2SignalHandler;

/**
 * @brief Test signal handler, will be executed alongside the main thread.
 *
 * @param sig Signal code.
 * @param logger_name Name of the rclcpp logger to use.
 * @param executor Pointer to main executor.
 * @param node Pointer to node being used.
 */
void custom_handler(
  int sig, std::string & logger_name, rclcpp::Executor::SharedPtr executor,
  Listener::SharedPtr node)
{
  //! Using this format, we can access just about everything of interest
  //! for this ROS 2 application, and do things depending on the
  //! signal we got
  if (sig == SIGTERM) {
    RCLCPP_WARN(rclcpp::get_logger(logger_name), "Canceling executor");
    executor->cancel(); //! This will cause spin to stop
  } else if (sig == SIGINT) {
    RCLCPP_INFO(node->get_logger(), "Got signal (%d)", sig);
  } else {
    return;
  }
}

int main(int argc, char ** argv)
{
  //! Create new context
  auto context = std::make_shared<rclcpp::Context>();

  //! Set context initialization options: we want it to be shut down on signal,
  //! to ease cleanup
  rclcpp::InitOptions init_options = rclcpp::InitOptions();
  init_options.shutdown_on_sigint = true;

  //! Initialize context
  context->init(argc, argv, init_options);

  //! This will be executed as last steps of shutdown, might give another
  //! degree of freedom in cleanup
  context->on_shutdown(
    [context]() -> void {
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(MODULE_NAME),
        "Context shut down: " << context->shutdown_reason()
      );
    }
  );

  //! Create startup options for node specifying context
  rclcpp::NodeOptions node_opts = rclcpp::NodeOptions();
  node_opts.context(context);

  //! Create listener node
  auto node = std::make_shared<Listener>(node_opts);

  //! Create a basic executor and add the node to it (we will need a pointer to it)
  //! Note that we must pass it some options to specify our custom context
  rclcpp::ExecutorOptions executor_opts = rclcpp::ExecutorOptions();
  executor_opts.context = context;
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_opts);
  executor->add_node(node);

  //! NOW YOU HAVE ALL POSSIBLE DATA TO SET SIGNAL HANDLERS
  //! Get global signal handler object instance
  SignalHandler & signal_handler = SignalHandler::get_global_signal_handler();

  //! Initialize it (note the different ways of passing handlers to it)
  signal_handler.init(
    context,
    "test_handler",
    std::bind(
      custom_handler,
      std::placeholders::_1,
      std::placeholders::_2,
      executor,
      node
    ),
    [node](int sig, siginfo_t * info, void * ucontext) -> void {
      UNUSED(info);
      UNUSED(ucontext);
      RCLCPP_INFO(
        node->get_logger(),
        "Hello from system handler for signal (%d)",
        sig);
    }
  );

  //! Install handlers for SIGINT, SIGTERM, ignore SIGHUP
  signal_handler.install(SIGINT);
  signal_handler.install(SIGTERM);
  signal_handler.ignore(SIGHUP);

  //! Spin the executor
  RCLCPP_WARN(
    rclcpp::get_logger(MODULE_NAME),
    "(%d) Now try one of SIGINT, SIGTERM, SIGHUP",
    getpid()
  );
  executor->spin();

  //! Finalize the signal handler and get out
  signal_handler.fini();

  exit(EXIT_SUCCESS);
}
