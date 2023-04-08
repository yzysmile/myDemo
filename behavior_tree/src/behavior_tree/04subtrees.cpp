# include "../../include/simple_nodes/04subtrees.hpp"

// Fallback下的子节点 有一个子节点返回BT::NodeStatus::SUCCESS即成功

// <Inverter>
//     <IsDoorClosed/>
// </Inverter>
// 以上三行相当于"IsDoorOpen"

// 节点 IsDoorClosed 自身若返回BT::NodeStatus::SUCCESS；
// 但其是Inverter的子节点，故反转结果为 BT::NodeStatus::FAILURE。

static const char* xml_text = R"(
 <root BTCPP_format="4">

    <BehaviorTree ID="MainTree">
        <Sequence>
            <Fallback>
                <Inverter>
                    <IsDoorClosed/>
                </Inverter>
                <SubTree ID="DoorClosed"/>
            </Fallback>
            <PassThroughDoor/>
        </Sequence>
    </BehaviorTree>

    <BehaviorTree ID="DoorClosed">
        <Fallback>
            <OpenDoor/>
            <RetryUntilSuccessful num_attempts="5">
                <PickLock/>
            </RetryUntilSuccessful>
            <SmashDoor/>
        </Fallback>
    </BehaviorTree>
    
</root>

)";


int main()
{
  BT::BehaviorTreeFactory factory;
  CrossDoor cross_door;
  cross_door.RegisterNodes(factory);

  
  factory.registerBehaviorTreeFromText(xml_text);

  // xml中包含 行为树 和 其子树
  // 要确定 哪一个 是 主要的，使用其 ID
  auto tree = factory.createTree("MainTree");

  // helper function to print the tree
  BT::printTreeRecursively(tree.rootNode());

  tree.tickRootWhileRunning();

  return 0;
}