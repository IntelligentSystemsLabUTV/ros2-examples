/**
 * AddTwoInts service client node.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * November 22, 2021
 */

#include <iostream>
#include <stdexcept>

#include <simple_service_cpp/simple_service.hpp>

/**
 * @brief Client node constructor.
 */
AddTwoIntsClient::AddTwoIntsClient()
: Node("client")
{
  //! Create a client object with create_client from the base class:
  //! this->create_client<INTERFACE_TYPE>(
  //!   SERVICE_NAME [string],
  //!   ...
  //! );
  client_ = this->create_client<AddTwoInts>("/examples/add_two_ints");

  RCLCPP_INFO(this->get_logger(), "Client initialized");
}

/**
 * @brief Calls the service with the given data, prints the response.
 *
 * @param a Request a.
 * @param b Request b.
 *
 * @throws RuntimeError
 */
void AddTwoIntsClient::call_srv(
  int a,
  int b,
  AddTwoIntsClient::SharedPtr node_ptr)
{
  //! Wait for the service to become available
  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    //! It is always good to check the status of the middleware while waiting
    //! for the service (bad things may happen otherwise)
    if (!rclcpp::ok()) {
      //! Best thing is to throw an exception, since middleware APIs cannot
      //! be called reliably if things are not ok
      throw std::runtime_error("Middleware crashed while waiting for service");
    }
    RCLCPP_WARN(this->get_logger(), "Service not available");
  }

  //! Create and populate the request (again, as a shared object)
  auto request = std::make_shared<AddTwoInts::Request>();
  request->set__a(a);
  request->set__b(b);

  //! Send the request, then wait for the response to come back
  //! Never, NEVER use sync'ed I/O for this (because of underlying threads)
  auto response = client_->async_send_request(request);
  //! Note: it's a FutureAndRequestId object

  //! And because of underlying jobs to do, since this is a single-threaded program,
  //! relinquish control to the middleware while waiting for the response to arrive
  //! (this creates a new single-threaded executor, adds the node to it, ...)
  //! In a multithreaded environment, we could just call get() on the future embedded in 'response'
  //! and trace exceptions
  //! Note: we could specify a timeout
  if (rclcpp::spin_until_future_complete(node_ptr, response) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    //! To access the value, we must access the future with get()
    //! It would block until the value was available, but the spin call above
    //! ensures us that we'll get here only when it is available
    RCLCPP_INFO(this->get_logger(), "Result: %ld", response.get()->sum);
  } else {
    //! Some cleanup is required to avoid memory leaks
    client_->remove_pending_request(response);
    RCLCPP_ERROR(this->get_logger(), "Service call failed");
  }
}

int main(int argc, char ** argv)
{
  // Parse input arguments
  if (argc < 3) {
    //! Note: why did we check for less than 3 arguments?
    std::cerr << "Usage:\n\tclient a b" << std::endl;
    exit(EXIT_FAILURE);
  }
  int a = std::stoi(argv[1]);
  int b = std::stoi(argv[2]);

  rclcpp::init(argc, argv);
  auto client_node = std::make_shared<AddTwoIntsClient>();

  //! Note: this time we don't spin, we just call a method offered by the node
  client_node->call_srv(a, b, client_node);

  // Just exit
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
