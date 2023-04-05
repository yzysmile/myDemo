# include <string>
# include <iostream>

# include "behaviortree_cpp_v3/bt_factory.h"

// "行为树" 中的各个节点可通过blackboard进行数据共享
// blackboard 中包含多个 entry, entry是一种 key/value pair 的数据结构

// outputPort和inputPort 分别用于单个节点 进行 数据写入blackboard 及 读取blackboard的数据 
// InputPort 能够读取blackboard 中的entry, OutputPort 能够 将entry写入blackboard.

// 写入blackboard的数据(entry)的value 在节点的tick()函数 以setOutput("port_name", "value")实现；
// 读取blackboard的数据(entry)的value 在节点的tick()函数 以auto msg = getInput("port_name")实现。

// 写入/读取blackboard的数据(entry)的key， 在xml 以"portname = {key}" 的格式进行设置；
class ThinkWhatToSay:public BT::SyncActionNode
{
  public:
  // 若一个Node中 包含ports,必须 使用以下的 签名格式的 构造函数
  ThinkWhatToSay(const std::string& name, const BT::NodeConfiguration& config):SyncActionNode(name, config)
  {

  }

  // 必须定义为 静态函数 (STATIC method).
  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<std::string>("text")};
  }

  BT::NodeStatus tick() override
  {
    // 名为"text"的OutputPort 将 value为"The answer is 42" 写入blackboard
    setOutput("text", "The answer is 42");
    return BT::NodeStatus::SUCCESS;
  }
};



class SaySomething:public BT::SyncActionNode
{
public:
  SaySomething(const std::string& name, const BT::NodeConfiguration& config):SyncActionNode(name, config)
  { 

  }
  
  static BT::PortsList providedPorts()
  {
    // InputPort: 读取blockport中的entry； OutputPort: 写入blockport中的entry
    // 该节点中有一个名为 "message" 的port，数据类型为 std::string
    return {BT::InputPort<std::string>("message")};
  }

  BT::NodeStatus tick() override
  {  
     // 名为"message"的port 的数据, 可以读取 blockboard中 指定key(xml中) 的 value
     auto msg = getInput<std::string>("message");

     if (!msg)
    {
      throw BT::RuntimeError("missing required input [message]: ", 
                              msg.error() );
    }
    // use the method value() to extract the valid message.
    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};