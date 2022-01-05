/**
 * Publisher definition.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * November 22, 2021
 */

#ifndef PUB_HPP
#define PUB_HPP

#include <rclcpp/rclcpp.hpp> //! rclcpp base library

#ifndef EXAMPLE
#include <std_msgs/msg/string.hpp> //! interface library that we'll use
#else
#include <ros2_examples_interfaces/msg/string.hpp>
#endif

#define PUB_PERIOD 300 // Publisher transmission time period [ms]

/**
 * Simple publisher node: transmits strings on a topic.
 */
//! Every node must extend publicly the Node base class
class Pub : public rclcpp::Node
{
public:
  //! There must always be a constructor, with arbitrary input arguments
  Pub();

//! ROS-specific members better be private
private:
  //! DDS endpoint, acting as a publisher
  //! Syntax is: rclcpp::Publisher<INTERFACE_TYPE>::SharedPtr OBJ;
#ifndef EXAMPLE
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
#else
  rclcpp::Publisher<ros2_examples_interfaces::msg::String>::SharedPtr publisher_;
#endif

  //! ROS-2 managed timer: enables one to set up a periodic job
  //! The job is coded in a callback, which better be a private method
  //! Syntax is: rclcpp::TimerBase::SharedPtr OBJ;
  //! Callback signature must be: void FUNC_NAME(void);
  rclcpp::TimerBase::SharedPtr pub_timer_;
  void pub_timer_callback(void);

  unsigned long pub_cnt_; // Marks messages
};

#endif
