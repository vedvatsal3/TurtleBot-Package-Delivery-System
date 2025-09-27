
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <fstream>
#include "secoro2_delivery_system/srv/request_delivery.hpp"

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("request_delivery_cli");
  auto client = node->create_client<secoro2_delivery_system::srv::RequestDelivery>("/request_delivery");

  std::cout << "Rooms / Staff:\n"
            << "TAB2.51: VK\n2.52: AO\n2.53: MN, SW\n2.54: NH\n2.55: AL\n2.56: KS\n2.57: meeting_room\n\n";

  std::cout << "Safety briefing:\n"
            << "- Robot carries up to 2 kg and 30x30x30 cm.\n"
            << "- Secure the package to prevent slipping or falling.\n"
            << "- Do not touch the robot while it is moving.\n"
            << "- Keep pathways clear during operation.\n"
            << "- Follow local lab safety rules at all times.\n";
  std::cout << "Proceed? [y/n]: ";
  std::string conf; std::getline(std::cin, conf);
  if(conf!="y" && conf!="Y"){
    std::cerr << "Aborted by user.\n";
    rclcpp::shutdown(); return 1;
  }

  std::string sname,sroom,rname,rroom;
  std::cout << "Sender name: "; std::getline(std::cin, sname);
  std::cout << "Sender room: "; std::getline(std::cin, sroom);
  std::cout << "Receiver name: "; std::getline(std::cin, rname);
  std::cout << "Receiver room: "; std::getline(std::cin, rroom);

  if (!client->wait_for_service(std::chrono::seconds(3))){
    std::cerr << "request_delivery service not available\n";
    rclcpp::shutdown(); return 2;
  }

  auto req = std::make_shared<secoro2_delivery_system::srv::RequestDelivery::Request>();
  req->sender_name = sname; req->sender_room = sroom;
  req->receiver_name = rname; req->receiver_room = rroom;
  auto fut = client->async_send_request(req);
  auto resp = fut.get();
  if (!resp->accepted){
    std::cerr << "Rejected: " << resp->message << "\n";
    rclcpp::shutdown(); return 3;
  }
  std::cout << "Assigned robot: " << resp->assigned_robot << "\n";
  std::cout << "Process ID: " << resp->process_id << "\n";
  std::cout << "PIN: " << resp->pin << "\n";
  std::cout << "Sender must sign in within 3 minutes using: ros2 run secoro2_delivery_system sender_client\n";
  std::cout << "Then receiver should be ready within 5 minutes using: ros2 run secoro2_delivery_system receiver_client\n";
  rclcpp::shutdown();
  return 0;
}
