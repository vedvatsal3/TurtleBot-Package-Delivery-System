
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include "secoro2_delivery_system/srv/signin.hpp"
#include "secoro2_delivery_system/srv/receiver_confirm.hpp"

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("receiver_client");
  auto signin = node->create_client<secoro2_delivery_system::srv::Signin>("/signin");
  auto confirm = node->create_client<secoro2_delivery_system::srv::ReceiverConfirm>("/receiver_confirm");

  std::string user, pid, pin;
  std::cout << "Username: "; std::getline(std::cin, user);
  std::cout << "Process ID: "; std::getline(std::cin, pid);
  std::cout << "PIN: "; std::getline(std::cin, pin);

  if (!signin->wait_for_service(std::chrono::seconds(3))) {
    std::cerr << "/signin not available\n"; rclcpp::shutdown(); return 1;
  }
  auto req = std::make_shared<secoro2_delivery_system::srv::Signin::Request>();
  req->process_id = pid; req->pin = pin; req->role = "receiver";
  auto resp = signin->async_send_request(req).get();
  if (!resp->ok){ std::cerr << "Signin failed: " << resp->message << "\n"; rclcpp::shutdown(); return 2; }

  std::cout << "Waiting for robot to arrive ... you'll be prompted to confirm pickup.\n";
  rclcpp::shutdown();
  return 0;
}
