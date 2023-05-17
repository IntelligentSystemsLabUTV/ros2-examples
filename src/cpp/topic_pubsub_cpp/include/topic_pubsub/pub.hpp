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

#include <std_msgs/msg/string.hpp> //! Interface library that we'll use

//! We'll see how to properly manage this kind of "parameters"
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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  //! ROS-2 managed timer: enables one to set up a periodic job
  //! The job is coded in a callback, which better be a private method
  //! Syntax is: rclcpp::TimerBase::SharedPtr OBJ;
  //! Callback signature must be: void FUNC_NAME(void);
  rclcpp::TimerBase::SharedPtr pub_timer_;
  void pub_timer_callback(void);

  unsigned long pub_cnt_; // Marks messages
};

#endif
