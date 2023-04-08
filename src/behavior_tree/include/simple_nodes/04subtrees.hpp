# include <string>
# include <iostream>

# include "behaviortree_cpp_v3/bt_factory.h"

class CrossDoor
{
public:
  inline BT::NodeStatus IsDoorClosed()
  { 
    std::cout << "IsDoorClosed" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return (!door_open_)? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  
  inline BT::NodeStatus PassThroughDoor()
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return door_open_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  
  // After 3 attempts, will open a locked door
  inline BT::NodeStatus PickLock()
  {
    if (pick_attempts_++ > 3){
        door_locked_ = false;
        door_open_ = true;
    }
    return door_open_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  

  inline BT::NodeStatus OpenDoor()
  {
    if (door_locked_){
       return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  inline BT::NodeStatus SmashDoor()
  {
    if(door_open_){
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
      door_open_ = false;
    }
    return BT::NodeStatus::SUCCESS;
  }
   
  void RegisterNodes(BT::BehaviorTreeFactory& factory)
  {
    // "..."中的为 节点名称, 与xml中登记的节点要相同。
    factory.registerSimpleCondition(
      "IsDoorClosed", std::bind(&CrossDoor::IsDoorClosed, this));
    factory.registerSimpleAction(
      "PassThroughDoor", std::bind(&CrossDoor::PassThroughDoor, this));
    factory.registerSimpleAction(
      "OpenDoor", std::bind(&CrossDoor::OpenDoor, this));
    factory.registerSimpleAction(
      "PickLock", std::bind(&CrossDoor::PickLock, this));
    factory.registerSimpleCondition(
      "SmashDoor", std::bind(&CrossDoor::SmashDoor, this));
  }
  
  void reset()
  {
    door_open_   = false;
    door_locked_ = true;
    pick_attempts_ = 0;
  }

private:
    bool door_open_   = false;
    bool door_locked_ = true;
    int pick_attempts_ = 0;
};
