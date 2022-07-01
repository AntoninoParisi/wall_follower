#include "actions.h"

using namespace std;
using namespace BT;
using namespace Actions;

/** Behavior Tree are used to create a logic to decide what
 * to "do" and when. For this reason, our main building blocks are
 * Actions and Conditions.
 */

static const char* xml_tree = R"(
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="Wall follower main sequence">	
		<Find_Wall name="Find wall"/>	
		<Align name="Align"/>
        <Follow_Wall name="Follow wall"/>			
        </Sequence>
     </BehaviorTree>
</root>
)";

int main()
{

    BehaviorTreeFactory factory;

    factory.registerSimpleAction("Find_Wall", bind(Find_Wall));
    factory.registerSimpleAction("Align", bind(Align));
    factory.registerSimpleAction("Follow_Wall", bind(Follow_Wall));

    auto tree = factory.createTreeFromText(xml_tree);

    cout << "Executing Wall Follower..." << endl; 

    // To "execute" a Tree you need to "tick" it.
    //tree.tickRoot();

    // Tick the tree until it reaches a terminal state
    NodeStatus status = NodeStatus::RUNNING;
    while (status == NodeStatus::RUNNING) {
        status = tree.tickRoot();
        rclcpp::sleep_for(chrono::milliseconds(5)); 
    }

    return 0;
}

