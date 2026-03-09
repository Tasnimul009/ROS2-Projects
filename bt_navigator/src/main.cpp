#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

// Shorthand alias — makes long type names readable
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;


// ═══════════════════════════════════════════════════════════
// INITIAL POSE PUBLISHER
// Replaces manual "2D Pose Estimate" click in RViz
// ═══════════════════════════════════════════════════════════
void publishInitialPose(rclcpp::Node::SharedPtr node)
{
  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

  auto publisher = node->create_publisher<PoseMsg>("/initialpose", 10);

  PoseMsg pose_msg;
  pose_msg.header.frame_id = "map";
  pose_msg.header.stamp = node->get_clock()->now();

  // Starting position of TurtleBot3 in turtlebot3_world
  pose_msg.pose.pose.position.x = 0.0;
  pose_msg.pose.pose.position.y = 0.0;
  pose_msg.pose.pose.orientation.w = 1.0;

  // Standard AMCL covariance values
  pose_msg.pose.covariance[0]  = 0.25;
  pose_msg.pose.covariance[7]  = 0.25;
  pose_msg.pose.covariance[35] = 0.06853891945200942;

  std::cout << "[INIT] Publishing initial pose..." << std::endl;
  for (int i = 0; i < 10; i++) {
    publisher->publish(pose_msg);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  std::cout << "[INIT] Initial pose set!" << std::endl;
}


// ═══════════════════════════════════════════════════════════
// NavigateTo — BT ACTION NODE
// ═══════════════════════════════════════════════════════════
class NavigateTo : public BT::StatefulActionNode
{
public:

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;  // 👈 clean, no multi-line template


  // ───────────────────────────────────────────────
  // Constructor
  // ───────────────────────────────────────────────
  NavigateTo(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("navigate_to_" + name);

    client_ = rclcpp_action::create_client<NavigateToPose>(
      node_, "navigate_to_pose"
    );

    RCLCPP_INFO(node_->get_logger(), "NavigateTo node ready!");
  }


  // ───────────────────────────────────────────────
  // Ports — reads x, y from XML
  // ───────────────────────────────────────────────
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y")
    };
  }


  // ───────────────────────────────────────────────
  // onStart — sends goal to Nav2 once
  // ───────────────────────────────────────────────
  BT::NodeStatus onStart() override
  {
    double x, y;
    getInput("x", x);
    getInput("y", y);

    RCLCPP_INFO(node_->get_logger(), "Navigating to x=%.2f y=%.2f", x, y);

    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(), "Nav2 not available!");
      return BT::NodeStatus::FAILURE;
    }

    NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node_->get_clock()->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.w = 1.0;

    auto future = client_->async_send_goal(goal_msg);
    rclcpp::spin_until_future_complete(node_, future);
    goal_handle_ = future.get();

    if (!goal_handle_) {
      RCLCPP_ERROR(node_->get_logger(), "Goal rejected by Nav2!");
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }


  // ───────────────────────────────────────────────
  // onRunning — checks every tick if goal is done
  // ───────────────────────────────────────────────
  BT::NodeStatus onRunning() override
  {
    auto future_result = client_->async_get_result(goal_handle_);

    auto spin_status = rclcpp::spin_until_future_complete(
      node_, future_result,
      std::chrono::milliseconds(100)
    );

    if (spin_status != rclcpp::FutureReturnCode::SUCCESS) {
      return BT::NodeStatus::RUNNING;
    }

    auto result = future_result.get();

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(node_->get_logger(), "Waypoint reached!");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Navigation failed!");
      return BT::NodeStatus::FAILURE;
    }
  }


  // ───────────────────────────────────────────────
  // onHalted — cancels goal if BT is interrupted
  // ───────────────────────────────────────────────
  void onHalted() override
  {
    RCLCPP_WARN(node_->get_logger(), "Halted — cancelling goal");
    client_->async_cancel_goal(goal_handle_);
  }
};


// ═══════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Step 1 — Publish initial pose (no RViz click needed!)
  auto init_node = rclcpp::Node::make_shared("init_pose_node");
  publishInitialPose(init_node);

  // Step 2 — Wait for AMCL to process pose
  std::cout << "[INIT] Waiting for AMCL to localize..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(7));

  // Step 3 — Register BT nodes
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<NavigateTo>("NavigateTo");

  // Step 4 — Load XML tree
  std::string package_path =
    ament_index_cpp::get_package_share_directory("bt_navigator");
  std::string xml_path = package_path + "/trees/navigator_tree.xml";

  std::cout << "[BT] Loading tree from: " << xml_path << std::endl;
  auto tree = factory.createTreeFromFile(xml_path);

  // Step 5 — Tick tree until done
  std::cout << "[BT] Starting patrol mission..." << std::endl;

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // Step 6 — Report result
  if (status == BT::NodeStatus::SUCCESS) {
    std::cout << "[BT] Patrol mission complete!" << std::endl;
  } else {
    std::cout << "[BT] Patrol mission failed!" << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}