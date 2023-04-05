# include "../../include/simple_nodes/your_first_behavior_tree.hpp"

int main()
{
  // We use the BehaviorTreeFactory to register our custom nodes
  BT::BehaviorTreeFactory factory;
  
  // The recommended way to create a Node is through inheritance.
  factory.registerNodeType<behaviortree::ApproachObject>("ApproachObject");

  // Registering a SimpleActionNode using a function pointer
  // Here we prefer to use a lambda,but you can use std::bind too
  factory.registerSimpleCondition("CheckBattery", std::bind(behaviortree::CheckBattery));

  // You can also create SimpleActionNodes using methods of a class.
  behaviortree::GripperInterface gripper;
  factory.registerSimpleAction("OpenGripper", std::bind(&behaviortree::GripperInterface::open, &gripper));
  factory.registerSimpleAction("CloseGripper", std::bind(&behaviortree::GripperInterface::close, &gripper));
  
  auto tree = factory.createTreeFromFile("/home/yzyrobot/myself_demo/src/behavior_tree/my_tree.xml");
  // auto tree = factory.createTreeFromFile("../../my_tree.xml");
  tree.tickRootWhileRunning();

  return 0;
}
