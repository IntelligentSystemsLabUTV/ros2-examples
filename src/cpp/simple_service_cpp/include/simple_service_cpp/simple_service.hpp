/**
 * AddTwoInts server and client nodes.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * November 22, 2021
 */

#ifndef SIMPLE_SERVICE_HPP
#define SIMPLE_SERVICE_HPP

#include <rclcpp/rclcpp.hpp>

#include <ros2_examples_interfaces/srv/add_two_ints.hpp>
//! ALWAYS INCLUDE THE INTERFACE .hpp HEADER!

//! Let's make things a little bit simpler here
using namespace ros2_examples_interfaces::srv;

/**
 * AddTwoInts server node.
 */
class AddTwoIntsServer : public rclcpp::Node
{
public:
  AddTwoIntsServer();

private:
  //! This object is a modified DDS endpoint that receives requests and,
  //! in addition, implements the ROS 2 server semantics to send responses
  //! The job to execute upon arrival must be coded in a related callback
  //! The syntax is:
  //! rclcpp::Service<INTERFACE_TYPE>::SharedPtr OBJ;
  rclcpp::Service<AddTwoInts>::SharedPtr server_;

  //! Service callback signature must be:
  //! void FUNC_NAME(const INTERFACE_TYPE::Request::SharedPtr ARG1,
  //!                const INTERFACE_TYPE::Response::SharedPtr ARG2);
  void add_two_ints_clbk(
    const AddTwoInts::Request::SharedPtr request,
    const AddTwoInts::Response::SharedPtr response);
};

/**
 * AddTwoInts client node.
 */
class AddTwoIntsClient : public rclcpp::Node
{
public:
  AddTwoIntsClient();

  //! This is just for us, not ROS-related, i.e. we could do without it (see below)
  void call_srv(int a, int b, AddTwoIntsClient::SharedPtr node_ptr);

private:
  //! This object is a modified DDS endpoint that sends requests and,
  //! in addition, implements the ROS 2 client semantics to receive responses
  //! The syntax is:
  //! rclcpp::Client<INTERFACE_TYPE>::SharedPtr OBJ;
  rclcpp::Client<AddTwoInts>::SharedPtr client_;
};
//! We could do without this: if our only intent is to call a ROS 2 service from an application,
//! we could just create a node, a service client, and call the service synchronously from
//! our 'main' function or wherever else we want

#endif
