
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include "secoro2_delivery_system/srv/signin.hpp"
#include "secoro2_delivery_system/srv/sender_confirm.hpp"

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sender_client");
  auto signin = node->create_client<secoro2_delivery_system::srv::Signin>("/signin");
  auto confirm = node->create_client<secoro2_delivery_system::srv::SenderConfirm>("/sender_confirm");

  std::string user, pid, pin;
  std::cout << "Username: "; std::getline(std::cin, user);
  std::cout << "Process ID: "; std::getline(std::cin, pid);
  std::cout << "PIN: "; std::getline(std::cin, pin);

  if (!signin->wait_for_service(std::chrono::seconds(3))) {
    std::cerr << "/signin not available\n"; rclcpp::shutdown(); return 1;
  }
  auto req = std::make_shared<secoro2_delivery_system::srv::Signin::Request>();
  req->process_id = pid; req->pin = pin; req->role = "sender";
  auto resp = signin->async_send_request(req).get();
  if (!resp->ok){ std::cerr << "Signin failed: " << resp->message << "\n"; rclcpp::shutdown(); return 2; }

  std::cout << "Waiting for robot to arrive ... you'll be prompted to confirm drop-on.\n";
  // The server will call back via /sender_confirm
  // We keep the node alive to handle service. In practice, this client only triggers sign-in.
  rclcpp::shutdown();
  return 0;
}
