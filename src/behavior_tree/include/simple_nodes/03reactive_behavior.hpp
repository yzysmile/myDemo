# include <string>
# include <iostream>

# include "behaviortree_cpp_v3/behavior_tree.h"

struct Pose2D
{
  double x, y, theta;
};

namespace chr = std::chrono;

namespace BT{
   template<>
   inline Pose2D convertFromString(BT::StringView str)
   {
      auto parts = BT::splitString(str, ';');
      if (parts.size()!= 3)
      {
        throw BT::RuntimeError("invalid input");
      }else{
        Pose2D output;
        output.x     = convertFromString<double>(parts[0]);
        output.y     = convertFromString<double>(parts[1]);
        output.theta = convertFromString<double>(parts[2]);
        return output;
      }
   }
}  // BT

class MoveBaseAction : public BT::StatefulActionNode
{
public:
  // 任意带有port的节点 构造函数必须使用以下签名
  MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config):StatefulActionNode(name, config)
  {

  }

  static BT::PortsList providedPorts()
  {
      // 该Node有一个读取blackboard中entry的port(可以在xml指定 这个port的value)
      return{BT::InputPort<Pose2D>("goal")};
  }

  
  BT::NodeStatus onStart() override
  {
    if (!getInput<Pose2D>("goal", goal_)){
        throw BT::RuntimeError("missing required input [goal]");
    }

    printf("[ MoveBase: SEND REQUEST ]. goal: x=%f y=%f theta=%f\n",
         goal_.x, goal_.y, goal_.theta);

    completion_time_ = chr::system_clock::now() + chr::milliseconds(220);

    return BT::NodeStatus::RUNNING;
  }
  
  BT::NodeStatus onRunning() override
  {
    // Pretend that we are checking if the reply has been received
    // you don't want to block inside this function too much time.
    std::this_thread::sleep_for(chr::milliseconds(10));

    // Pretend that, after a certain amount of time,
    // we have completed the operation
    if(chr::system_clock::now() >= completion_time_)
    {
        std::cout << "[ MoveBase: FINISHED ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    printf("[ MoveBase: ABORTED ]");
  }

  private:
    Pose2D goal_;
    chr::system_clock::time_point completion_time_;
};