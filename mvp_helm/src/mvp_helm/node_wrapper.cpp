#include "mvp_helm/node_wrapper.h"

#include <memory>
#include <string>
#include <vector>

namespace helm{

NodeWrapper::NodeWrapper(
  const std::string & node_name,
  const std::string & ns,
  const rclcpp::NodeOptions & options)
: Node(node_name, ns, options)
{

    // declare some parameters that could shared by all the node
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This parameter is mine!";

    this->declare_parameter("my_parameter", "world", param_desc);

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "hi")};
    this->set_parameters(all_new_parameters);

    //! TODO: some function to print something ?

}

NodeWrapper::~NodeWrapper()
{
    RCLCPP_INFO(get_logger(), "Destroying");

    //! TODO: cleanup something ?
}


} // end of helm