#include "../../include/simple_nodes/01blackboard_and_ports.hpp"
#include "../../include/simple_nodes/03reactive_behavior.hpp"
#include "../../include/simple_nodes/05port_remapping.hpp"

static const char* xml_text = R"(
<root main_tree_to_execute = "MainTree">

    <BehaviorTree ID="MainTree">
        <Sequence name="main_sequence">
            
            <SetBlackboard output_key="move_goal" value="1;2;3" />
            <SubTree ID="MoveRobot" target="move_goal" 
                                    output="move_result" />
            <SaySomething message="{move_result}"/>
        </Sequence>

    </BehaviorTree>

    <BehaviorTree ID="MoveRobot">
        <Fallback name="move_robot_main">
            <Sequence>
                <MoveBase       goal="{target}"/>
                <SetBlackboard output_key="output" value="mission accomplished" />
            </Sequence>
            <ForceFailure>
                <SetBlackboard output_key="output" value="mission failed" />
            </ForceFailure>
        </Fallback>
    </BehaviorTree>

</root>
 )";

 int main()
 {
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<MoveBaseAction>("MoveBase");

    factory.registerBehaviorTreeFromText(xml_text);
    auto tree = factory.createTree("MainTree");

    tree.tickRootWhileRunning();
    
    // let's visualize some information about the current state of the blackboards.
    std::cout << "\n------ First BB ------" << std::endl;
    tree.blackboard_stack[0]->debugMessage();
    std::cout << "\n------ Second BB------" << std::endl;
    tree.blackboard_stack[1]->debugMessage();

    return 0;
 }