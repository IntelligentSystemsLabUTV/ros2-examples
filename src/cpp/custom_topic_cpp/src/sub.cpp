/**
 * Subscriber definition and implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * November 22, 2021
 */

#include <iostream>

#include <custom_topic_cpp/sub.hpp>

/**
 * @brief Creates a Sub node.
 */
//! Call the base class constructor providing a string embedding the node name
//! Initialize other members at will
Sub::Sub()
: Node("subscriber_node")
{
  //! This time we use an explicit QoS policy by creaitng a QoS object
  //! Syntax is: rclcpp::QoS obj_name(QUEUE_DEPTH)
  //! Then, we modify the individual parameters using the setter methods and the enums
  //! For a complete reference see the qos.hpp header from rclcpp (try to open it via VS Code!)
  rclcpp::QoS topic_qos(rclcpp::KeepLast(1));
  topic_qos.best_effort(); //! Equivalenty: topic_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  topic_qos.durability_volatile(); //! Equivalenty: topic_qos.durability(rclcpp::DurabilityPolicy::Volatile);

  //! Initialize a subscriber with create_subscription from the base class:
  //! this->create_subscription<INTERFACE_TYPE>(
  //!   TOPIC_NAME [string],
  //!   SUBSCRIPTION_QoS (...),
  //!   CALLBACK_WRAPPER (with captures and argument placeholders)
  //!   (...)
  //! );
  subscriber_ = this->create_subscription<ros2_examples_interfaces::msg::String>(
    "/publisher_node/examples/test_topic",
    topic_qos,
    std::bind(
      &Sub::msg_callback,
      this,
      std::placeholders::_1));

  //! Logging macro used to deliver a message to the logging subsystem, INFO level
  RCLCPP_INFO(this->get_logger(), "Subscriber initialized");
}

/**
 * @brief Echoes a new message.
 *
 * @param msg New message.
 */
void Sub::msg_callback(const ros2_examples_interfaces::msg::String::SharedPtr msg)
{
  //! Get the data simply by accessing the object member as the interface specifies
  RCLCPP_INFO(this->get_logger(), msg->data.c_str());
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
  auto sub_node = std::make_shared<Sub>();

  //! This automatically creates a default, single-threaded executor, adds to it
  //! the jobs defined by the new node, and makes the current thread tend to that
  rclcpp::spin(sub_node);

  //! Whatever happens now, does so only after rclcpp::spin has returned

  //! Shut down the global context, the DDS participant and all its
  //! endpoints, effectively terminating the connection to the DDS layer
  //! (note: this doesn't necessarily destroy the corresponding objects)
  rclcpp::shutdown();

  // Just exit
  std::cout << "Subscriber terminated" << std::endl;
  exit(EXIT_SUCCESS);
}
