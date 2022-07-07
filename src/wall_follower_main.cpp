#include "actions.h"
#include "wall_follower.h"

//using namespace BT;

/** Behavior Tree are used to create a logic to decide what
 * to "do" and when. For this reason, our main building blocks are
 * Actions and Conditions.
 */

const char *xml_tree = R"(
                        <root main_tree_to_execute="MainTree">
                            <BehaviorTree ID="MainTree">
                              <Sequence name="MainTree_seq">	
                                <SubTree ID="Wall_follower"/>
                                <SubTree ID="Corner_Handler"/>
                              </Sequence>  
                            </BehaviorTree>

                            <BehaviorTree ID="Wall_follower">
                              <SequenceStar name="Wall_follower_seq">	
                                <Find_Wall name="Find wall"/>	
                                <Side_Choice name="Side choice"/>                              
                                <Align name="Align"/>
                                <Follow_Wall name="Follow a wall"/> 
                              </SequenceStar>
                            </BehaviorTree> 

                            <BehaviorTree ID="Corner_Handler">
                              <SequenceStar name="Corner_Handler_Seq">
                                <Side_Empty name="Is the side empty?"/>
                                <Follow_Corner name="Follow a corner"/> 
                              </SequenceStar>
                            </BehaviorTree> 
                        </root>
                        )";    


/*
<RetryUntilSuccessful num_attempts="999">
</RetryUntilSuccessful>  
*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Wall_Follower>(xml_tree));
  cout << "Wall Follower Ends" << endl;
  return 0;
}
