/**
 * Parametric node code.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * December 3, 2021
 */

#include <chrono>
#include <limits>

//! This is only required if you want to set parameters from within the code!
//! See topic callback below!
#include <rclcpp/node_interfaces/node_parameters.hpp>

#include "../include/parameters_example/parametric_node.hpp"

/**
 * Creates a parametric publisher using the value of the parameter.
 */
ParametricPub::ParametricPub()
: Node("parametric_pub")
{
  // Create publisher
  num_publisher_ = this->create_publisher<Int32>(
    "/ros2_examples/parameter",
    rclcpp::QoS(10)
  );

  // Create timer
  pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(
      &ParametricPub::pub_routine,
      this
    )
  );

  // Create subscriber
  param_set_sub_ = this->create_subscription<Int32>(
    "/ros2_examples/set_parameter",
    rclcpp::QoS(10),
    std::bind(
      &ParametricPub::param_set_clbk,
      this,
      std::placeholders::_1)
  );

  //! Create descriptors for each parameter
  //! This is not mandatory, but improves readability when inspecting the system
  //! via CLI commands and introspection tools
  //! See the rcl_interfaces/msg/ParameterDescriptor and ParameterType interface definitions
  //! to understand what each of its memebers does, and here:
  //! https://answers.ros.org/question/378427/how-is-the-name-field-of-parameterdescriptor-used-by-declare_parameter/
  //! This system relies on messages since it is entirely built on services
  //! The descriptor won't be updated unless we do so explicitly, even when we
  //! change the parameter's value
  rcl_interfaces::msg::IntegerRange param_range{};
  param_range.set__from_value(0L);
  param_range.set__to_value(2147483647L);
  param_range.set__step(1UL);
  param_descriptor_.set__name("number");
  param_descriptor_.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER); //! From ParameterType.msg
  param_descriptor_.set__description("Example integer parameter.");
  param_descriptor_.set__additional_constraints(
    "32-bit signed positive integer representation limits apply.");
  param_descriptor_.set__read_only(false);
  param_descriptor_.set__dynamic_typing(false);
  param_descriptor_.set__integer_range({param_range});

  // Register parameter set callback
  //! This traces all ROS APIs too, so must be registered first
  param_clbk_handle_ = this->add_on_set_parameters_callback(
    std::bind(
      &ParametricPub::param_clbk,
      this,
      std::placeholders::_1)
  );

  //! Declare each parameter
  //! Simplest syntax is, for any type:
  //! declare_parameter(NAME_STRING, DEFAULT_VALUE, DESCRIPTOR);
  this->declare_parameter("number", 1, param_descriptor_);

  //! Update internal data with new parameter (note the cast!)
  pub_num_ = this->get_parameter("number").as_int();
  //! This happens to be a best practice: subsequent accesses will be faster
  //! since it is now a class member, and we don't need to call the middleware
  //! to retrieve the value it stores each time
  //! This also needs in-house callbacks for setting it at runtime, see below
}

/**
 * Simple callback that publishes a new message.
 */
void ParametricPub::pub_routine()
{
  // Publish new message
  Int32 new_msg{};
  new_msg.set__data(pub_num_);
  num_publisher_->publish(new_msg);
  RCLCPP_INFO(this->get_logger(), "Published: %d", new_msg.data);
}

/**
 * Sets a new value for the parameter read from the topic.
 *
 * @param msg Int32 message to parse.
 */
void ParametricPub::param_set_clbk(const Int32::SharedPtr msg)
{
  // Create a new Parameter object to pass to the API
  rclcpp::node_interfaces::ParameterInfo new_param_info = { //! It's a struct
    rclcpp::ParameterValue(msg->data), //! This member must be of this type!
    param_descriptor_
  };
  rclcpp::Parameter new_param(new_param_info);
  //! This way it shall inherit name, type, ranges and so on

  // Try to set the new parameter and publish the result of the operation
  rcl_interfaces::msg::SetParametersResult res = this->set_parameter(new_param);
  //! Prepare to a template hell to extract values from Parameter objects,
  //! or use the as_TYPE methods, your choice
  if (res.successful) {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Succesfully set parameter to: " << new_param.get_value<rclcpp::ParameterType::PARAMETER_INTEGER>() << " (" << res.reason <<
        ")");
  } else {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "Failed to set parameter to: " << new_param.get_value<rclcpp::ParameterType::PARAMETER_INTEGER>() << " (" << res.reason <<
        ")");
  }
}

//! THIS IS CALLED AFTER THE INTERNAL PARAMETER UPDATE IS PERFORMED!
//! Both the internal parameter update check and this have to succeed for the
//! parameter to be updated
//! Range checks have already been performed before we get to this,
//! so they would be redundant here
//! Here we just store the new parameter value
//! ref.: https://roboticsbackend.com/ros2-rclcpp-parameter-callback/
/**
 * Called each time a parameter is set, allows us to react to the change.
 *
 * @param params Vector of parameters for which a change has been requested.
 */
rcl_interfaces::msg::SetParametersResult ParametricPub::param_clbk(
  const std::vector<rclcpp::Parameter> & params)
{
  // Initialize result object
  rcl_interfaces::msg::SetParametersResult res{};
  res.set__successful(false);
  res.set__reason("Invalid parameters");

  // Look for valid parameters to update, and populate result accordingly
  for (const rclcpp::Parameter & p : params) {
    if ((p.get_name() == "number") && (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)) {
      RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Requested parameter change to: " << p.value_to_string());
      int new_val = p.as_int();
      //! To prove that this is executed after ROS internals we add an additional
      //! condition; try to set the parameter to 0
      if (new_val == 0)
      {
        res.set__successful(false);
        res.set__reason("Callback considers 0 invalid");
        return res;
      }
      pub_num_ = new_val; //! Watch out for concurrency!
      res.set__successful(true);
      res.set__reason("");
      break;
    }
  }

  return res;
}
