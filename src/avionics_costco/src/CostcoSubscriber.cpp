/**
 * @file CostcoSubscriber.cpp
 * @author Eliot Abramo, Matas Jones
 * @brief 
 * @date 2025-07-03
 */

#include "CostcoSubscriber.h"
#include "Cosco.hpp"

std::unique_ptr<Cosco> cosco_sub;

/**
 * @brief Constructor for the costco publisher, declare the subscribers on the
 * topics and read data from ros to be sent on serial
 */
CostcoSubscriber::CostcoSubscriber() : Node("costco_subscriber") {

  RCLCPP_INFO(this->get_logger(), "Creating CostcoSubscriber");

  this->declare_parameter<std::string>("port_name", "");
  std::string port_name = this->get_parameter("port_name").as_string();
  RCLCPP_INFO(this->get_logger(), "port_name_2: %s", port_name.c_str());

  cosco_sub = std::make_unique<Cosco>(port_name, 115200);

  // Instantiate the subscribers
  // !!! topic name != message name (for LEDMessage, why? ask Gio)
  // These subscribers are binded with a handle such that when a message is
  // received, the handle is called

  this->servo_request_ =
      this->create_subscription<custom_msg::msg::ServoRequest>(
          ("/EL/servo_req"), 10,
          std::bind(&CostcoSubscriber::ServoRequestHandler, this,
                    std::placeholders::_1));
  
  this->mass_request_hd_ =
    this->create_subscription<custom_msg::msg::MassRequestHD>(
        ("/EL/mass_req_hd"), 10,
        std::bind(&CostcoSubscriber::MassRequestHDHandler, this,
                  std::placeholders::_1));

  this->mass_request_drill_ =
  this->create_subscription<custom_msg::msg::MassRequestDrill>(
      ("/EL/mass_req_drill"), 10,
      std::bind(&CostcoSubscriber::MassRequestDrillHandler, this,
                std::placeholders::_1));


}

/**
 * @brief Destroy the costco subscriber, delete all the pointers and set address
 * to nullptr
 */
CostcoSubscriber::~CostcoSubscriber() {
  RCLCPP_INFO(this->get_logger(), "Deleting CostcoSubscriber");
}

// Handle for the Servo message
void CostcoSubscriber::ServoRequestHandler(const custom_msg::msg::ServoRequest::SharedPtr msg) {
  ServoRequest servoRequesteMsg;
  servoRequesteMsg.id = msg->id;
  servoRequesteMsg.increment = msg->increment;
  servoRequesteMsg.zero_in = msg->zero_in;
  RCLCPP_INFO(this->get_logger(), "Servo received");

  cosco_sub->sendServo(&servoRequesteMsg, servoRequesteMsg.id);
  RCLCPP_INFO(this->get_logger(), "Servo sent to ESP32");
}

// Handle for the Servo message
void CostcoSubscriber::MassRequestHDHandler(const custom_msg::msg::MassRequestHD::SharedPtr msg) {
  MassRequestHD MassRequesteMsg;
  MassRequesteMsg.scale = msg->scale;
  MassRequesteMsg.tare = msg->tare;
  RCLCPP_INFO(this->get_logger(), "Mass Request received");

  cosco_sub->sendMassRequestHD(&MassRequesteMsg);
  RCLCPP_INFO(this->get_logger(), "Mass Request HD sent to ESP32");
  // RCLCPP_INFO(this->get_logger(), "id: %u, scale: %f, tare: %d", MassRequesteMsg.id,MassRequesteMsg.scale,MassRequesteMsg.tare);
}

void CostcoSubscriber::MassRequestDrillHandler(const custom_msg::msg::MassRequestDrill::SharedPtr msg) {
  MassRequestDrill MassRequesteMsg;
  MassRequesteMsg.scale = msg->scale;
  MassRequesteMsg.tare = msg->tare;
  RCLCPP_INFO(this->get_logger(), "Mass Request received");

  cosco_sub->sendMassRequestDrill(&MassRequesteMsg);
  RCLCPP_INFO(this->get_logger(), "Mass Request Drill sent to ESP32");
  // RCLCPP_INFO(this->get_logger(), "id: %u, scale: %f, tare: %d", MassRequesteMsg.id,MassRequesteMsg.scale,MassRequesteMsg.tare);
}
