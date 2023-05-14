/**
 * Dummy publisher/server source code.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 9, 2022
 */

#include "../../include/namespaces_examples/topic_ns/pub_srv.hpp"

#define UNUSED(arg) (void)(arg)

/**
 * @brief Creates a new dummy publisher/server.
 */
DummyPubSrv::DummyPubSrv()
: Node("dummy_node")
{
  //! If we put no additional characters in a topic/service name, it'll
  //! be considered as a "relative name" and put directly in the outermost
  //! namespace (see docs)
  //! Instead, here we explicitly require placing topics and services in the
  //! "private namespace" by prepending "~/", and the expansion will be:
  //! /DEFAULT_NAMESPACE (if any)/NODE/name
  pub_ = this->create_publisher<Bool>(
    "~/dummy_topic",
    rclcpp::QoS(10)
  );
  //! For the sake of this example, we'll never need to publish on this topic

  server_ = this->create_service<Trigger>(
    "~/dummy_srv",
    std::bind(
      &DummyPubSrv::server_clbk,
      this,
      std::placeholders::_1,
      std::placeholders::_2)
  );
}

/**
 * @brief Dummy server callback.
 *
 * @param req Request to parse.
 * @param resp Response to populate.
 */
void DummyPubSrv::server_clbk(
  Trigger::Request::SharedPtr req,
  Trigger::Response::SharedPtr resp)
{
  UNUSED(req);
  resp->set__success(true);
  resp->set__message("Service was called");
}
