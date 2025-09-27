
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <random>
#include <mutex>
#include <condition_variable>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <irobot_create_msgs/action/dock.hpp>
#include "secoro2_delivery_system/srv/request_delivery.hpp"
#include "secoro2_delivery_system/srv/signin.hpp"
#include "secoro2_delivery_system/srv/sender_confirm.hpp"
#include "secoro2_delivery_system/srv/receiver_confirm.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using Dock = irobot_create_msgs::action::Dock;
using DockGoalHandle = rclcpp_action::ClientGoalHandle<Dock>;

struct Waypoint { double x; double y; double yaw; };

static const std::vector<std::string> AVAILABLE_ROBOTS = {"george", "fred"};
static std::unordered_set<std::string> BUSY_ROBOTS;

static const std::unordered_map<std::string, Waypoint> ROOM_LOCATIONS = {
  {"room_251", {-4.5,  1.7,  0.0}}, {"room_252", { 1.3,  1.7,  0.0}},
  {"room_253", { 4.57, 1.7,  0.0}}, {"room_254", { 4.57,-1.7,  0.0}},
  {"room_255", { 1.3, -1.7,  0.0}}, {"room_256", {-1.3, -1.7,  0.0}},
  {"room_257", {-4.5, -1.7,  0.0}}, {"dock_george",{-2.25, 2.4, 0.0}},
  {"dock_fred", {-2.55, 2.2,  0.0}}
};

struct DoorPassage { Waypoint pre; Waypoint post; };
static const std::unordered_map<std::string, DoorPassage> DOOR_PASSAGES = {
  {"room_251", {{-4.5,  0.5,  0.0},{-4.5,  1.7,  0.0}}},
  {"room_252", {{ 1.3,  0.5,  0.0},{1.3,  1.7,  0.0}}},
  {"room_253", {{ 4.57, 0.5,  0.0},{ 4.57, 1.7,  0.0}}},
  {"room_254", {{ 4.57,-0.5,  0.0},{4.57,-1.7,  0.0}}},
  {"room_255", {{ 1.3, -0.5,  0.0},{1.3, -1.7,  0.0}}},
  {"room_256", {{-1.3, -0.5,  0.0},{-1.3, -1.7,  0.0}}},
  {"room_257", {{-4.5, -0.5,  0.0},{-4.5, -1.7,  0.0}}},
  {"dock_george", {{-4.5, 0.5, 0.0},{-2.25, 2.4, 0.0}}},
  {"dock_fred",   {{-1.3, -0.5, 0.0},{-2.55, -2.2, 0.0}}}
};

static std::optional<Waypoint> resolve_room(const std::string& key){
  auto it = ROOM_LOCATIONS.find(key);
  if (it==ROOM_LOCATIONS.end()) return std::nullopt;
  return it->second;
}

class DeliveryBTServer : public rclcpp::Node {
public:
  DeliveryBTServer() : Node("delivery_bt_server")
  {
    monitor_pub_ = this->create_publisher<std_msgs::msg::String>("/monitor/events", 10);

    // Battery subscriptions per robot
    for (auto& ns : AVAILABLE_ROBOTS){
      auto sub = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/" + ns + "/battery_state",
        rclcpp::QoS(10),
        [this, ns](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
          if (!msg) return;
          battery_pct_[ns] = msg->percentage * 100.0f;  // 0..1 -> percent
        }
      );
      battery_subs_.push_back(sub);
      battery_pct_[ns] = 100.0; // default
    }

    // Services
    request_srv_ = this->create_service<secoro2_delivery_system::srv::RequestDelivery>(
      "/request_delivery",
      std::bind(&DeliveryBTServer::on_request_delivery, this, std::placeholders::_1, std::placeholders::_2));

    signin_srv_ = this->create_service<secoro2_delivery_system::srv::Signin>(
      "/signin",
      std::bind(&DeliveryBTServer::on_signin, this, std::placeholders::_1, std::placeholders::_2));

    sender_confirm_client_   = this->create_client<secoro2_delivery_system::srv::SenderConfirm>("/sender_confirm");
    receiver_confirm_client_ = this->create_client<secoro2_delivery_system::srv::ReceiverConfirm>("/receiver_confirm");

    RCLCPP_INFO(this->get_logger(), "DeliveryBTServer ready.");
  }

private:
  struct Process {
    std::string id, pin;
    std::string sender_user, sender_room;
    std::string receiver_user, receiver_room;
    std::string robot;
    bool sender_signed{false};
    bool completed{false};
  };
  std::unordered_map<std::string, Process> processes_;
  std::mutex mtx_;
  std::unordered_map<std::string,
    rclcpp_action::Client<NavigateToPose>::SharedPtr> nav_clients_;
  std::unordered_map<std::string,
    rclcpp_action::Client<Dock>::SharedPtr> dock_clients_;

  // ROS
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr monitor_pub_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> battery_subs_;
  std::unordered_map<std::string, double> battery_pct_;

  rclcpp::Service<secoro2_delivery_system::srv::RequestDelivery>::SharedPtr request_srv_;
  rclcpp::Service<secoro2_delivery_system::srv::Signin>::SharedPtr signin_srv_;
  rclcpp::Client<secoro2_delivery_system::srv::SenderConfirm>::SharedPtr sender_confirm_client_;
  rclcpp::Client<secoro2_delivery_system::srv::ReceiverConfirm>::SharedPtr receiver_confirm_client_;

  // Helpers
  void monitor_note_(const std::string& txt){
    std_msgs::msg::String m; m.data = txt;
    monitor_pub_->publish(m);
  }

  static std::string gen_id_(){
    static std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<int> d(1,99999);
    return "delivery" + std::to_string(d(rng));
  }
  static std::string gen_pin_(){
    static std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<int> d(1000,9999);
    return std::to_string(d(rng));
  }

  bool has_robot_available_(std::string& picked){
    // choose first non-busy with battery > 35
    for (auto& r : AVAILABLE_ROBOTS){
      if (BUSY_ROBOTS.count(r)) continue;
      if (battery_pct_[r] < 35.0) continue;
      picked = r;
      return true;
    }
    return false;
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr
  nav_client_for_(const std::string& ns)
  {
    auto it = nav_clients_.find(ns);
    if (it != nav_clients_.end()) return it->second;

    // Action name is namespaced by robot, e.g. /george/navigate_to_pose
    auto client = rclcpp_action::create_client<NavigateToPose>(
        this->shared_from_this(), "/" + ns + "/navigate_to_pose");

    // Wait briefly so we don’t fire into the void
    if (!client->wait_for_action_server(5s)) {
      RCLCPP_WARN(this->get_logger(),
                  "NavigateToPose action server not available for ns '%s'", ns.c_str());
    }
    nav_clients_[ns] = client;
    return client;
  }

  rclcpp_action::Client<Dock>::SharedPtr
  dock_client_for_(const std::string& ns)
  {
    auto it = dock_clients_.find(ns);
    if (it != dock_clients_.end()) return it->second;

    auto client = rclcpp_action::create_client<Dock>(
        this->shared_from_this(), "/" + ns + "/dock");

    if (!client->wait_for_action_server(5s)) {
      RCLCPP_WARN(this->get_logger(),
                  "Dock action server not available for ns '%s'", ns.c_str());
    }
    dock_clients_[ns] = client;
    return client;
  }


  // --- Services ---
  void on_request_delivery(
    const std::shared_ptr<secoro2_delivery_system::srv::RequestDelivery::Request> req,
    std::shared_ptr<secoro2_delivery_system::srv::RequestDelivery::Response> resp)
  {
    // Validate rooms
    if (!resolve_room(req->sender_room) || !resolve_room(req->receiver_room)){
      resp->accepted = false;
      resp->message = "Invalid room.";
      return;
    }
    std::string robot;
    if (!has_robot_available_(robot)){
      resp->accepted = false;
      resp->message = "No robot available.";
      return;
    }
    auto id = gen_id_();
    auto pin = gen_pin_();

    {
      std::lock_guard<std::mutex> lk(mtx_);
      processes_[id] = Process{
        id, pin, req->sender_name, req->sender_room,
        req->receiver_name, req->receiver_room,
        robot, false, false
      };
      BUSY_ROBOTS.insert(robot);
    }

    resp->accepted = true;
    resp->assigned_robot = robot;
    resp->process_id = id;
    resp->pin = pin;
    resp->message = "Provide these to sender & receiver. Sender must sign in within 3 minutes.";

    // Spawn a thread to execute BT for this process
    std::thread([this, id](){
      run_process_bt_(id);
    }).detach();
  }

  void on_signin(
    const std::shared_ptr<secoro2_delivery_system::srv::Signin::Request> req,
    std::shared_ptr<secoro2_delivery_system::srv::Signin::Response> resp)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = processes_.find(req->process_id);
    if (it == processes_.end()){
      resp->ok = false; resp->message = "Unknown process_id"; return;
    }
    if (it->second.pin != req->pin){
      resp->ok = false; resp->message = "Wrong PIN"; return;
    }
    if (req->role == "sender"){
      it->second.sender_signed = true;
      resp->ok = true; resp->message = "Sender signed in.";
      monitor_note_("[sender] signed-in for " + req->process_id);
      return;
    }
    resp->ok = true; resp->message = "OK";
  }

  // --- BT Execution ---
  struct BTContext {
    DeliveryBTServer* self;
    std::string pid;
    std::string robot;
    std::string sender_user, sender_room;
    std::string receiver_user, receiver_room;
  };

  bool navigate_sync_(const std::string& ns, const Waypoint& wp){
    
    auto client = nav_client_for_(ns);

    // (optional) ensure server is up
    if (!client->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "[%s] NavigateToPose server not available", ns.c_str());
      return false;
    }

    NavigateToPose::Goal goal;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = wp.x;
    pose.pose.position.y = wp.y;
    pose.pose.orientation.w = 1.0;  // TODO: set quaternion from yaw if needed
    goal.pose = pose;

    auto send_future = client->async_send_goal(goal);
    if (send_future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "[%s] NavigateToPose send_goal timed out", ns.c_str());
      return false;
    }
    auto gh = send_future.get();
    if (!gh) {
      RCLCPP_ERROR(this->get_logger(), "[%s] Failed to send NavigateToPose goal", ns.c_str());
      return false;
    }

    auto result_future = client->async_get_result(gh);
    if (result_future.wait_for(180s) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "[%s] NavigateToPose result timed out", ns.c_str());
      return false;
    }
    auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(this->get_logger(), "[%s] NavigateToPose failed (%d)", ns.c_str(), (int)wrapped.code);
      return false;
    }
    return true;
    }

    bool dock_sync_(const std::string& ns){
      auto client = dock_client_for_(ns);

      if (!client->wait_for_action_server(5s)) {
        RCLCPP_ERROR(this->get_logger(), "[%s] Dock action server not available", ns.c_str());
        return false;
      }

      Dock::Goal goal;  // fill if your action needs fields

      auto send_future = client->async_send_goal(goal);
      if (send_future.wait_for(5s) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "[%s] Dock send_goal timed out", ns.c_str());
        return false;
      }
      auto gh = send_future.get();
      if (!gh) {
        RCLCPP_ERROR(this->get_logger(), "[%s] Failed to send Dock goal", ns.c_str());
        return false;
      }

      auto result_future = client->async_get_result(gh);
      if (result_future.wait_for(120s) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "[%s] Dock result timed out", ns.c_str());
        return false;
      }
      auto wrapped = result_future.get();
      if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(this->get_logger(), "[%s] Dock failed (%d)", ns.c_str(), (int)wrapped.code);
        return false;
      }
      return true;
    }
  bool sender_confirm_(const std::string& msg, std::chrono::milliseconds timeout){
    if (!sender_confirm_client_->wait_for_service(10s)) return false;
    auto req = std::make_shared<secoro2_delivery_system::srv::SenderConfirm::Request>();
    req->message = msg;
    auto fut = sender_confirm_client_->async_send_request(req);
    if (fut.wait_for(timeout) != std::future_status::ready) return false;
    auto resp = fut.get();
    return resp->confirmed;
  }

  bool receiver_confirm_(const std::string& msg, std::chrono::milliseconds timeout){
    if (!receiver_confirm_client_->wait_for_service(10s)) return false;
    auto req = std::make_shared<secoro2_delivery_system::srv::ReceiverConfirm::Request>();
    req->message = msg;
    auto fut = receiver_confirm_client_->async_send_request(req);
    if (fut.wait_for(timeout) != std::future_status::ready) return false;
    auto resp = fut.get();
    return resp->confirmed;
  }

  // Leaf nodes
  class AssignRobotNode : public BT::SyncActionNode {
  public:
    AssignRobotNode(const std::string& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg) {}
    static BT::PortsList providedPorts(){ return {}; }
    BT::NodeStatus tick() override {
      auto& ctx = *config().blackboard->get<BTContext*>("ctx");
      // robot already assigned in process creation
      return BT::NodeStatus::SUCCESS;
    }
  };

  class InitializeProcessNode : public BT::SyncActionNode {
  public:
    InitializeProcessNode(const std::string& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg) {}
    static BT::PortsList providedPorts(){ return {}; }
    BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
  };

  class AwaitSenderSigninNode : public BT::StatefulActionNode {
  public:
    AwaitSenderSigninNode(const std::string& name, const BT::NodeConfiguration& cfg) : BT::StatefulActionNode(name, cfg) {}
    static BT::PortsList providedPorts(){ return {}; }
    BT::NodeStatus onStart() override { start_ = std::chrono::steady_clock::now(); return BT::NodeStatus::RUNNING; }
    BT::NodeStatus onRunning() override {
      auto& ctx = *config().blackboard->get<BTContext*>("ctx");
      auto* self = ctx.self;
      std::lock_guard<std::mutex> lk(self->mtx_);
      auto it = self->processes_.find(ctx.pid);
      if (it != self->processes_.end() && it->second.sender_signed) return BT::NodeStatus::SUCCESS;
      return BT::NodeStatus::RUNNING;
    }
    void onHalted() override {}
  private:
    std::chrono::steady_clock::time_point start_;
  };

  class NavigateToDoorNode : public BT::SyncActionNode {
  public:
    NavigateToDoorNode(const std::string& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg) {}
    static BT::PortsList providedPorts(){ return { BT::InputPort<std::string>("room") }; }
    BT::NodeStatus tick() override {
      auto& ctx = *config().blackboard->get<BTContext*>("ctx");
      auto* self = ctx.self;
      auto room = getInput<std::string>("room").value();
      auto it = DOOR_PASSAGES.find(room);
      if (it == DOOR_PASSAGES.end()){
        // if no door passage, treat as success (skip to inside)
        return BT::NodeStatus::SUCCESS;
      }
      return self->navigate_sync_(ctx.robot, it->second.pre) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  };

  class NavigateIntoRoomNode : public BT::SyncActionNode {
  public:
    NavigateIntoRoomNode(const std::string& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg) {}
    static BT::PortsList providedPorts(){ return { BT::InputPort<std::string>("room") }; }
    BT::NodeStatus tick() override {
      auto& ctx = *config().blackboard->get<BTContext*>("ctx");
      auto* self = ctx.self;
      auto room = getInput<std::string>("room").value();
      auto wp = resolve_room(room);
      if (!wp) return BT::NodeStatus::FAILURE;
      return self->navigate_sync_(ctx.robot, *wp) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  };

  class NotifyNode : public BT::SyncActionNode {
  public:
    NotifyNode(const std::string& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg) {}
    static BT::PortsList providedPorts(){ return { BT::InputPort<std::string>("text") }; }
    BT::NodeStatus tick() override {
      auto& ctx = *config().blackboard->get<BTContext*>("ctx");
      auto* self = ctx.self;
      auto text = getInput<std::string>("text").value();
      self->monitor_note_(text);
      return BT::NodeStatus::SUCCESS;
    }
  };

  class AwaitDropOnNode : public BT::SyncActionNode {
  public:
    AwaitDropOnNode(const std::string& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg) {}
    static BT::PortsList providedPorts(){ return { BT::InputPort<std::string>("user") }; }
    BT::NodeStatus tick() override {
      auto& ctx = *config().blackboard->get<BTContext*>("ctx");
      auto* self = ctx.self;
      bool ok = self->sender_confirm_("Please confirm drop-on [y]: ", 5min);
      return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  };

  class AwaitPickupNode : public BT::SyncActionNode {
  public:
    AwaitPickupNode(const std::string& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg) {}
    static BT::PortsList providedPorts(){ return { BT::InputPort<std::string>("user") }; }
    BT::NodeStatus tick() override {
      auto& ctx = *config().blackboard->get<BTContext*>("ctx");
      auto* self = ctx.self;
      bool ok = self->receiver_confirm_("Please confirm pickup [y]: ", 5min);
      return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
  };

  class QueueHandlerNode : public BT::SyncActionNode {
  public:
    QueueHandlerNode(const std::string& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg) {}
    static BT::PortsList providedPorts(){ return {}; }
    BT::NodeStatus tick() override {
      // Placeholder to accept compatible waiting requests; not implemented fully
      return BT::NodeStatus::SUCCESS;
    }
  };

  class CompleteProcessNode : public BT::SyncActionNode {
  public:
    CompleteProcessNode(const std::string& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg) {}
    static BT::PortsList providedPorts(){ return {}; }
    BT::NodeStatus tick() override {
      auto& ctx = *config().blackboard->get<BTContext*>("ctx");
      auto* self = ctx.self;
      std::lock_guard<std::mutex> lk(self->mtx_);
      auto it = self->processes_.find(ctx.pid);
      if (it != self->processes_.end()){
        it->second.completed = true;
        BUSY_ROBOTS.erase(it->second.robot);
      }
      self->monitor_note_("[delivery] " + ctx.pid + " completed by " + ctx.robot);
      return BT::NodeStatus::SUCCESS;
    }
  };

  void run_process_bt_(const std::string& pid){
    // Prepare BT context
    BTContext ctx;
    ctx.self = this;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      auto p = processes_.at(pid);
      ctx.pid = p.id;
      ctx.robot = p.robot;
      ctx.sender_user = p.sender_user;
      ctx.sender_room = p.sender_room;
      ctx.receiver_user = p.receiver_user;
      ctx.receiver_room = p.receiver_room;
    }

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<AssignRobotNode>("AssignRobot");
    factory.registerNodeType<InitializeProcessNode>("InitializeProcess");
    factory.registerNodeType<AwaitSenderSigninNode>("AwaitSenderSignin");
    factory.registerNodeType<NavigateToDoorNode>("NavigateToDoor");
    factory.registerNodeType<NavigateIntoRoomNode>("NavigateIntoRoom");
    factory.registerNodeType<NotifyNode>("Notify");
    factory.registerNodeType<AwaitDropOnNode>("AwaitDropOnConfirmation");
    factory.registerNodeType<AwaitPickupNode>("AwaitPickupConfirmation");
    factory.registerNodeType<QueueHandlerNode>("QueueHandler");
    factory.registerNodeType<CompleteProcessNode>("CompleteProcess");

    // Load tree text from package share: here, embedded minimal for simplicity
    std::string tree_path = this->declare_parameter<std::string>("bt_path", "");
    std::string xml_text;
    if (!tree_path.empty()){
      // load from file
      std::ifstream ifs(tree_path);
      xml_text.assign((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    } else {
      xml_text = R"(<?xml version="1.0"?>
<root main_tree_to_execute="DeliveryRoot">
  <BehaviorTree ID="DeliveryRoot">
    <Sequence>
      <AssignRobot/>
      <InitializeProcess/>
      <Timeout msec="180000">
        <AwaitSenderSignin/>
      </Timeout>
      <SubTree ID="GoToSender"/>
      <SubTree ID="GoToReceiver"/>
      <CompleteProcess/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="GoToSender">
    <Sequence>
      <NavigateToDoor room="{sender_room}"/>
      <Fallback>
        <NavigateIntoRoom room="{sender_room}"/>
        <Sequence>
          <Notify text="Door closed. Please come to the door and place package."/>
          <Timeout msec="300000">
            <AwaitDropOnConfirmation user="{sender_user}"/>
          </Timeout>
        </Sequence>
      </Fallback>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="GoToReceiver">
    <Sequence>
      <Parallel success_threshold="1" failure_threshold="1">
        <Sequence>
          <NavigateToDoor room="{receiver_room}"/>
          <Fallback>
            <NavigateIntoRoom room="{receiver_room}"/>
            <Sequence>
              <Notify text="Door closed. Please come to the door to pick up."/>
              <Timeout msec="300000">
                <AwaitPickupConfirmation user="{receiver_user}"/>
              </Timeout>
            </Sequence>
          </Fallback>
        </Sequence>
        <QueueHandler/>
      </Parallel>
    </Sequence>
  </BehaviorTree>
</root>)";
    }

    auto tree = factory.createTreeFromText(xml_text);

    // Blackboard vars
    tree.rootBlackboard()->set("ctx", &ctx);
    tree.rootBlackboard()->set("sender_room", ctx.sender_room);
    tree.rootBlackboard()->set("receiver_room", ctx.receiver_room);
    tree.rootBlackboard()->set("sender_user", ctx.sender_user);
    tree.rootBlackboard()->set("receiver_user", ctx.receiver_user);

    // Tick loop
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    rclcpp::Rate rate(10);
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING){
      status = tree.tickRoot();
      //rclcpp::spin_some(this->get_node_base_interface());
      rate.sleep();
    }
    if (status != BT::NodeStatus::SUCCESS){
      monitor_note_("[delivery] " + pid + " ended with status != SUCCESS; canceling and releasing robot");
      std::string robot;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = processes_.find(pid);
        if (it != processes_.end()) { robot = it->second.robot; BUSY_ROBOTS.erase(it->second.robot); }
      }
      if (!robot.empty()) {
        auto dockkey = std::string("dock_") + robot;
        auto dwp = resolve_room(dockkey);
        if (dwp) { (void) navigate_sync_(robot, *dwp); }
        (void) dock_sync_(robot);
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeliveryBTServer>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);        
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
