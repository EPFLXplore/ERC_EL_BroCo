/**
 * @file CostcoMain.cpp
 * @author Matas Jones
 * @brief ROS2 main file for avionics node
 * @date 2025-07-03
 * 
 */

#include "CostcoPublisher.h"
#include "CostcoSubscriber.h"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief This is a generic main function that creates a publisher and
 * subscriber node
 *
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto subscriber_node = std::make_shared<CostcoSubscriber>();
  auto publisher_node = std::make_shared<CostcoPublisher>();

  // Create a MultiThreadedExecutor (it will use multiple threads)
  rclcpp::executors::MultiThreadedExecutor executor;

  // Add your nodes to the executor
  executor.add_node(subscriber_node);
  executor.add_node(publisher_node);

  RCLCPP_INFO(publisher_node->get_logger(), "Creating Publisher Node");
  RCLCPP_INFO(subscriber_node->get_logger(), "Creating Subscriber Node");

  // Spin the executor which conatains all the nodes
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

/**
 * ros2 topic pub /EL/dust_sensor custom_msg/msg/DustSensor "{id: 0, angle: 60.0, is_zero: 0}" --once
 *
 */