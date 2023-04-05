# include <string>
# include <iostream>

# include "behaviortree_cpp_v3/bt_factory.h"

namespace behaviortree{

// Example of custom SyncActionNode (synchronous action)
// without ports.
class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string& name) :
      BT::SyncActionNode(name, {})
  {}

  // You must override the virtual function tick()
  // tick()函数前包含了 该节点具体要执行的行为，并且返回 节点状态
  BT::NodeStatus tick() override
  {
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

BT::NodeStatus CheckBattery(){
  std::cout << "[ Battery: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}



class GripperInterface
{
public:
  GripperInterface():open_(true)
  {

  }

  BT::NodeStatus open()
  {
    open_ = true;
    std::cout << "GripperInterface::open" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus close()
  {
    std::cout << "GripperInterface::close" << std::endl;
    open_ = false;
    return BT::NodeStatus::SUCCESS;
  }

private:
  bool open_;
};


}  //  behaviortree

