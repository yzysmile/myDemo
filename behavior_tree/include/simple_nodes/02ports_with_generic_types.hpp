# include <string>
# include <iostream>

# include "behaviortree_cpp_v3/bt_factory.h"


struct Position2D
    {
      double x;
      double y;
    };

// 这里的namespace必须要使用BT,为啥？
namespace BT{
// 使用模板特化将 std::string 转化为 struct Position2D
   template <> 
   inline Position2D convertFromString(BT::StringView str)
   {
        // We expect real numbers separated by semicolons(分号)
        auto parts = BT::splitString(str, ';');
        if (parts.size() != 2)
        {
            throw BT::RuntimeError("invalid input)");
        }
        else{
            Position2D output;
            // We can use the specialization convertFromString<double>()
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            return output;
        }
   }
}   // behaviortree
    
class CalculateGoal : public BT::SyncActionNode
{
public:
  CalculateGoal(const std::string& name, const BT::NodeConfiguration& config):SyncActionNode(name, config)
  {

  }
   
  // 这个node中有一个 可将entry(entry的value类型是 Position2D)写入blackboard的port，port的名字为"goal"。
  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<Position2D>("goal")};
  }

  // 该BT::Node被 tick时 要执行的函数
  BT::NodeStatus tick() override
  {
    Position2D mygoal = {1.1, 2.3};
    // 设置entry的value为 mygoal
    setOutput("goal", mygoal);
    return BT::NodeStatus::SUCCESS;
  }
};

class PrintTarget : public BT::SyncActionNode
{
public:
  PrintTarget(const std::string& name, const BT::NodeConfiguration& config):SyncActionNode(name, config)
  {

  }
  
  static BT::PortsList providedPorts()
  {
    // 这个node中有一个 可读blackboard中entry的port，port的名字为"target"。
    const char* description = "Simply print the target on console...";
    return {BT::InputPort<Position2D>("target", description)};
  }

  BT::NodeStatus tick() override
  {
    // 根据entey的key值(在xml中设置)，读取blackboard中entry。
    auto res = getInput<Position2D>("target");
    if (!res)
    {
      throw BT::RuntimeError("error reading port [target]:", res.error());
    }
    Position2D goal = res.value();
    printf("Target positions: [ %.1f, %.1f ]\n", goal.x, goal.y);
    return BT::NodeStatus::SUCCESS;
  }
  
  
};