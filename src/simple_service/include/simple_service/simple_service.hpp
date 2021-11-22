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
#include <example_interfaces/srv/add_two_ints.hpp>
//! ALWAYS INCLUDE THE INTERFACE .hpp HEADER!

//! Let's make things a little bit simpler here
using namespace example_interfaces::srv;

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

  //! This is just for us, not ROS-related
  void call_srv(int a, int b, AddTwoIntsClient::SharedPtr node_ptr);

private:
  //! This object is a modified DDS endpoint that sends requests and,
  //! in addition, implements the ROS 2 client semantics to receive responses
  //! The syntax is:
  //! rclcpp::Client<INTERFACE_TYPE>::SharedPtr OBJ;
  rclcpp::Client<AddTwoInts>::SharedPtr client_;
};
//! We could do without this! Explain why: ...

#endif
