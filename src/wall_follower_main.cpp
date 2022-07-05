#include "actions.h"

using namespace std;
using namespace BT;

/** Behavior Tree are used to create a logic to decide what
 * to "do" and when. For this reason, our main building blocks are
 * Actions and Conditions.
 */

/*
const char *xml_tree = R"(
                    <root main_tree_to_execute="MainTree">
                        <BehaviorTree ID="MainTree">
                          <SequenceStar name="Wall follower main sequence">	
                            <Find_Wall name="Find wall"/>	
                            <Side_Choice name="Side choice"/>                              
                            <Align name="Align"/>
                            <Follow_Wall name="Follow wall"/>
                          </SequenceStar>
                        </BehaviorTree>
                    </root>
                    )";        
*/

const char *xml_tree = R"(
                        <root main_tree_to_execute="MainTree">
                            <BehaviorTree ID="MainTree">
                              <ReactiveSequence>
                                <SequenceStar name="Wall follower main sequence">	
                                  <Find_Wall name="Find wall"/>	
                                  <Side_Choice name="Side choice"/>                              
                                  <Align name="Align"/>
                                  <Follow_Wall name="Follow wall"/>
                                  <SubTree ID="IsSlimWall"/>
                                </SequenceStar>
                              </ReactiveSequence>
                            </BehaviorTree>
                            <BehaviorTree ID="IsSlimWall">
                                <ReactiveSequence>
                                    <SequenceStar name="slim wall check">
                                      <IsSlimWall name="Is slim wall"/>
                                      <Align name="Align"/>
                                      <Follow_Wall name ="slim wall"/>
                                    </SequenceStar>
                                </ReactiveSequence>
                              </BehaviorTree> 
                        </root>
                        )";                        

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Wall_Follower>(xml_tree));
  cout << "Wall Follower Ends" << endl;
  
  return 0;
}
