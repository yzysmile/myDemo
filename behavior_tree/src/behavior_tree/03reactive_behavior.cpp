# include <string>
# include <iostream>
# include <chrono>

# include "../../include/simple_nodes/00your_first_behavior_tree.hpp"
# include "../../include/simple_nodes/01blackboard_and_ports.hpp"
# include "../../include/simple_nodes/03reactive_behavior.hpp"

static const char* xml_text = R"(
 <root>
     <BehaviorTree>
        <ReactiveSequence>
            <BatteryOK/>
            <Sequence>
                <SaySomething   message="mission started..." />
                <MoveBase           goal="1;2;3"/>
                <SaySomething   message="mission completed!" />
            </Sequence>
        </ReactiveSequence>
     </BehaviorTree>
 </root>

)";

int main()
{
  using std::chrono::milliseconds;

  BT::BehaviorTreeFactory factory;
  factory.registerSimpleCondition("BatteryOK", std::bind(behaviortree::CheckBattery));
  factory.registerNodeType<MoveBaseAction>("MoveBase");
  factory.registerNodeType<SaySomething>("SaySomething");
  
  // auto tree = factory.createTreeFromText(xml_text);
  auto tree = factory.createTreeFromText(xml_text);

  std::cout << "--- ticking\n";
  BT::NodeStatus status = tree.tickRoot();
  std::cout << "--- status: " << BT::toStr(status) << "\n\n";

  while(status == BT::NodeStatus::RUNNING)
  {   
      std::cout << "while ticking\n";
      tree.sleep(std::chrono::milliseconds(100));

      std::cout << "--- ticking\n";
      status = tree.tickRoot();
      std::cout << "--- status: " << BT::toStr(status) << "\n\n";
  }

  return 0;
}

// 预期
// --- ticking
// [ Battery: OK ]
// Robot says: mission started...
// --- status: RUNNING

// --- ticking
// [ Battery: OK ]
// [ MoveBase: SEND REQUEST ]. goal: x=1.0 y=2.0 theta=3.0
// --- status: RUNNING

// --- ticking
// [ Battery: OK ]
// --- status: RUNNING

// --- ticking
// [ Battery: OK ]
// [ MoveBase: FINISHED ]
// Robot says: mission completed!
// --- status: SUCCESS




// 实际
// --- ticking
// [ Battery: OK ]
// Robot says: mission started...
// [ MoveBase: SEND REQUEST ]. goal: x=1.000000 y=2.000000 theta=3.000000
// --- status: RUNNING

// while ticking
// --- ticking
// [ Battery: OK ]
// --- status: RUNNING

// while ticking
// --- ticking
// [ Battery: OK ]
// [ MoveBase: FINISHED ]
// Robot says: mission completed!
// --- status: SUCCESS