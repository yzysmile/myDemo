# include "../../include/simple_nodes/blackboard_and_ports.hpp"

int main()
{
  BT::BehaviorTreeFactory factory;
  // ("SaySomething") 和 xml要一致
  factory.registerNodeType<SaySomething>("SaySomething");
  factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");

  auto tree = factory.createTreeFromFile("/home/yzyrobot/myself_demo/src/behavior_tree/blackboard_and_ports.xml");
  tree.tickRootWhileRunning();
  return 0;
}