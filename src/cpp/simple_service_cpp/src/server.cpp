/**
 * AddTwoInts service server node.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * November 22, 2021
 */

#include <iostream>

#include <simple_service_cpp/simple_service.hpp>

/**
 * @brief AddTwoInts server constructor.
 */
AddTwoIntsServer::AddTwoIntsServer()
: Node("server")
{
  //! Create a server object with create_service from the base class:
  //! this->create_service<INTERFACE_TYPE>(
  //!   SERVICE_NAME [string],
  //!   CALLBACK_WRAPPER,
  //!   ...
  //! );
  //! Note: we could have a '~' here, too, we just omit it for the sake of this example
  server_ = this->create_service<AddTwoInts>(
    "/examples/add_two_ints",
    //! Why do we need two placeholders this time? ...
    std::bind(
      &AddTwoIntsServer::add_two_ints_clbk,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Server initialized");
}

/**
 * @brief Adds two given integers and returns the sum.
 *
 * @param request Message with the two integers to add.
 * @param response Response message to populate.
 */
void AddTwoIntsServer::add_two_ints_clbk(
  const AddTwoInts::Request::SharedPtr request,
  const AddTwoInts::Response::SharedPtr response)
{
  //! Note that this has void return type: all that has to be done is to populate
  //! the appropriate fields in the response message
  response->set__sum(request->a + request->b);

  RCLCPP_INFO(
    this->get_logger(), "%ld + %ld = %ld",
    request->a,
    request->b,
    response->sum);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto server_node = std::make_shared<AddTwoIntsServer>();
  rclcpp::spin(server_node);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
