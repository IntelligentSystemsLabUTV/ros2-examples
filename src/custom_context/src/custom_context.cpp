/**
 * Example code to start a node in a custom context.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 13, 2022
 */

#include <iostream>
#include <unistd.h>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>

/* Will tell if a traced signal was delivered */
sig_atomic_t term = 0;

/**
 * @brief Context shutdown callback.
 */
//! Simple void function
void shutdown_callback()
{
  std::cout << "Context is being shut down" << std::endl;
}

/**
 * @brief Basic signal handler.
 *
 * @param sig Delivered signal code.
 */
void sig_handler(int sig)
{
  //! This has to be very quick, so apart from some very specific things let's
  //! just notify the main thread that a signal was delivered
  term = 1;
  std::cout << "Got signal: " << sig << std::endl;
}

int main(int argc, char ** argv)
{
  // Register signal handlers for SIGINT and SIGTERM
  struct sigaction sig_act;
  sig_act.sa_handler = sig_handler;
  sig_act.sa_flags = 0;
  sig_act.sa_restorer = NULL;
  if (sigfillset(&(sig_act.sa_mask))) {
    std::cerr << "Failed to set sig_act.sa_mask" << std::endl;
    perror("sigfillset");
    exit(EXIT_FAILURE);
  }
  if (sigaction(SIGINT, &sig_act, NULL) || sigaction(SIGTERM, &sig_act, NULL)) {
    std::cerr << "Failed to register signal handlers" << std::endl;
    perror("sigaction");
    exit(EXIT_FAILURE);
  }

  //! First we create a new context (i.e. DDS participant + ROS 2 stuff)
  rclcpp::Context::SharedPtr custom_context =
    std::make_shared<rclcpp::Context>();

  //! Then we set initialization options for it
  //! The only thing we can specify is whether this context has to be shut down
  //! on SIGINT
  rclcpp::InitOptions custom_init_options = rclcpp::InitOptions();
  custom_init_options.shutdown_on_sigint = false;

  //! Now we can initialize the context like when we call rclcpp::init
  custom_context->init(argc, argv, custom_init_options);

  //! We can even add a callback to be called when it is shut down
  //! From the documentation:
  //! These callbacks will be called in the order they are added as the second
  //! to last step in shutdown(). When shutdown occurs due to the signal
  //! handler, these callbacks are run asynchronoulsy in the dedicated singal
  //! handling thread. Also, shutdown() may be called from the destructor of
  //! this function. Therefore, it is not safe to throw exceptions from these
  //! callbacks. Instead, log errors or use some other mechanism to indicate an
  //! error has occurred. On shutdown callbacks may be registered before init
  //! and after shutdown, and persist on repeated init's.
  custom_context->on_shutdown(shutdown_callback);

  //! And now we can create nodes like we did, provided that we specify our
  //! custom context instead of letting contructors look for the global one
  //! For this we need a new object that holds options
  rclcpp::NodeOptions node_opts = rclcpp::NodeOptions();
  node_opts.context(custom_context);
  auto node = std::make_shared<rclcpp::Node>(
    "custom_node",
    node_opts);

  // Just go to sleep until a signal is delivered
  std::cout << getpid() << " going to sleep, please kill me" << std::endl;
  pause(); //! This might as well be a spin of some executor
  //! The Launch System will bring up many processes: the one you must target
  //! with kill commands is the one whose command embeds the full executable
  //! file path; for convenience, its PID will now be printed
  //! You can also test what happens if you start this with a launch file:
  //! by default you'd see less output unless proper configurations are made,
  //! and you should generally see becoming active:
  //! - A Python Launch System process...
  //! - ... which spawns a shell process...
  //! - ... which spawns this process

  // If a traced signal was delivered, shut down the context
  //! Here check if a signal was delivered and perform application-related,
  //! long-running cleanup tasks
  if (term) {
    bool done = custom_context->shutdown("Traced signal delivered");
    if (!done) {
      std::cerr << "Failed to shut down context" << std::endl;
      exit(EXIT_FAILURE);
    }
  }
  if (!custom_context->is_valid()) {
    std::cout << "Context shut down: " << custom_context->shutdown_reason() <<
      std::endl;
  }

  exit(EXIT_SUCCESS);
}
