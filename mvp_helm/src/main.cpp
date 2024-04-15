
#include <memory>

#include "mvp_helm/helm.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<helm::Helm>();

  node->initialize();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
