#include <bits/stdc++.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
using namespace std;
// -----------------------------------------------
// NODE 1: SayHello
// -----------------------------------------------
class SayHello : public BT::SyncActionNode
{
public:
  SayHello(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    cout << "[SayHello] Hello from Behaviour Tree!" << endl;
    return BT::NodeStatus::SUCCESS;
  }
};

// -----------------------------------------------
// NODE 2: CheckBattery
// -----------------------------------------------
class CheckBattery : public BT::SyncActionNode
{
public:
  CheckBattery(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    int battery = 80;
    if (battery > 20) {
      cout << "[CheckBattery] Battery OK: " << battery << "%" << endl;
      return BT::NodeStatus::SUCCESS;
    } else {
      cout << "[CheckBattery] Battery LOW!" << endl;
      return BT::NodeStatus::FAILURE;
    }
  }
};

// -----------------------------------------------
// MAIN
// -----------------------------------------------
int main(int argc, char** argv)
{
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<SayHello>("SayHello");
  factory.registerNodeType<CheckBattery>("CheckBattery");

  // Load tree from EXTERNAL xml file — not hardcoded!
  auto tree = factory.createTreeFromFile(
    "/home/tasz/ros2_ws/src/bt_patrol/trees/patrol_tree.xml"
  );

  cout << "--- Ticking the Tree ---" << endl;
  tree.tickRootWhileRunning();

  return 0;
}