#include "actions.h"
#include "wall_follower.h"

//using namespace BT;

/** Behavior Tree are used to create a logic to decide what
 * to "do" and when. For this reason, our main building blocks are
 * Actions and Conditions.
 */

//Sequence if a child return FAILURE      that the entire sequence is restarted from the first child of the list.
//SequenceStar if a child return FAILURE  the next time the sequence is ticked, the same child is ticked again. Previous sibling, which returned SUCCESS already, are not ticked again.
//In both case the sequence return SUCCESS if the last child return SUCCESS


const char *xml_tree = R"(
                        <root main_tree_to_execute="MainTree">
                            <BehaviorTree ID="MainTree">                                                                    
                              <Fallback name="Main_Sequence">
                                <SubTree ID="Parallel_Handler"/>
                                <Rewind name="Rewind"/>
                              </Fallback>                                                                               	                                                         
                            </BehaviorTree>

                            <BehaviorTree ID="Parallel_Handler">
                              <Sequence>                                     
                                <Wait_Key name="Is key pressed?"/>                                      
                                <SubTree ID="Wall_follower"/>
                              </Sequence>
                            </BehaviorTree>

                            <BehaviorTree ID="Wall_follower">
                                <Sequence name="Wall_follower_seq">
                                  <Sequence name="Wall_follower_basic_seq">	
                                    <Find_Wall name="Find wall"/>	
                                    <Side_Choice name="Side choice"/>                              
                                    <Align name="Align"/>
                                    <Follow_Wall name="Follow a wall"/> 
                                  </Sequence>
                                  <SubTree ID="Corner_Handler"/>                               
                                </Sequence> 
                            </BehaviorTree> 

                            <BehaviorTree ID="Corner_Handler">
                              <Fallback name="Corner_Handler_Seq">
                                <Side_Occupied name="Is the side Occupied?"/>
                                <Follow_Corner name="Follow a corner"/> 
                              </Fallback>  
                            </BehaviorTree> 

                        </root>
                        )";    
   


/*
<RetryUntilSuccessful num_attempts="999">
</RetryUntilSuccessful>  

<Repeat num_cycles="1000">

<Parallel success_threshold="1">

*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //while (rclcpp::ok())
  rclcpp::spin(std::make_shared<Wall_Follower>(xml_tree));
  cout << "Wall Follower Ends" << endl;
  return 0;
}