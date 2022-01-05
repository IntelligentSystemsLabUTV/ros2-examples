/**
 * Publisher definition and implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * November 22, 2021
 */

#include <iostream>

#include "../include/topic_pubsub/pub.hpp"

/**
 * @brief Creates a Pub node.
 */
//! Call the base class constructor providing a string embedding the node name
//! Initialize other members at will
Pub::Pub()
: Node("publisher_node"),
  pub_cnt_(0)
{
  //! Initialize a publisher with create_publisher from the base class:
  //! this->create_publisher<INTERFACE_TYPE>(
  //!   TOPIC_NAME [string],
  //!   PUBLISH_QoS (...),
  //!   ...
  //! );
  //! This object will be used later on to publish messages
#ifndef EXAMPLE
  publisher_ = this->create_publisher<std_msgs::msg::String>(
#else
  publisher_ = this->create_publisher<ros2_examples_interfaces::msg::String>(
#endif
    "/examples/test_topic",
    rclcpp::QoS(10));

  //! Create and activate a timer with create_wall_timer from the base class
  //! providing an std::chrono::duration as the period and a call wrapper for
  //! the callback, capturing the node object
  //! Since this callback must be void, the wrapper has no arguments specified
  pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(PUB_PERIOD),
    std::bind(
      &Pub::pub_timer_callback,
      this));

  //! Logging macro used to deliver a message to the logging subsystem, INFO level
  RCLCPP_INFO(this->get_logger(), "Publisher initialized");
}

/**
 * @brief Publishes a message on timer occurrence.
 */
void Pub::pub_timer_callback(void)
{
  // Build the new message
  std::string new_data = "Hello ";
  new_data.append(std::to_string(pub_cnt_) + ".");

  //! Create a new message of the specific interface type
  //! It is better to initialize it as empty as below
#ifndef EXAMPLE
  std_msgs::msg::String new_msg{};
#else
  ros2_examples_interfaces::msg::String new_msg{};
#endif

  //! Populate the data field of the message using its setter method
  new_msg.set__data(new_data);

  //! Publish the new message by calling the publish method of the publisher member
  publisher_->publish(new_msg);

  // Log something
  pub_cnt_++;
  RCLCPP_INFO(this->get_logger(), "Published message %lu", pub_cnt_);
}

int main(int argc, char ** argv)
{
  //! This automatically creates the global context->DDS participant for this application
  //! and parses all ROS-specific input arguments eventually passed to the new process
  //! (and installs signal handlers)
  rclcpp::init(argc, argv);

  //! Since it's all made of shared pointers, initialize the new node as such
  //! (note: here we're calling an actual constructor but obtaining a smart pointer
  //! to the new object)
  auto pub_node = std::make_shared<Pub>();

  //! This automatically creates a default, single-threaded executor, adds to it
  //! the jobs defined by the new node, and makes the current thread tend to that
  rclcpp::spin(pub_node);

  //! Whatever happens now, does so only after rclcpp::spin has returned

  //! Shut down the global context, the DDS participant and all its
  //! endpoints, effectively terminating the connection to the DDS layer
  //! (note: this doesn't necessarily destroy the corresponding objects)
  rclcpp::shutdown();

  // Just exit
  std::cout << "Publisher terminated" << std::endl;
  exit(EXIT_SUCCESS);
}
