# include "../../include/simple_nodes/02ports_with_generic_types.hpp"

static const char* xml_text = R"(

 <root BTCPP_format="4" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <CalculateGoal goal="{GoalPosition}" />
            <PrintTarget   target="{GoalPosition}" />
            <SetBlackboard output_key="OtherGoal" value="-1;3"/>
            <PrintTarget   target="{OtherGoal}" />
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main()
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<CalculateGoal>("CalculateGoal");
  factory.registerNodeType<PrintTarget>("PrintTarget");

  auto tree = factory.createTreeFromText(xml_text);
  // 解析错误???
  // auto tree = factory.createTreeFromText("/home/yzyrobot/myself_demo/src/behavior_tree/02ports_with_generic_types.xml");
  tree.tickRootWhileRunning();

  return 0;
}


/* Expected output:

    Target positions: [ 1.1, 2.3 ]
    Converting string: "-1;3"
    Target positions: [ -1.0, 3.0 ]
*/
