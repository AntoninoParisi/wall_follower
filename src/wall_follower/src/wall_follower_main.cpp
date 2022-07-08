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

// IDEA del BUG: nelle SequenceStar sembra che finche un nodo da failure questo si ripete ma in realtà non è cosi.
//               Infatti in realtà la sequenza termina e viene fatta ripartire, questa volta non da capo, ma da dove eravamo rimasti.
//               Cercando di far ripetere il nodo di un ramo, questo verra si ripetuto ma anche tutti i rami prima. 


const char *xml_tree = R"(
                        <root main_tree_to_execute="MainTree">
                            <BehaviorTree ID="MainTree">
                              <Sequence name="MainTree_safe_seq">
                                <Parallel success_threshold="1">
                                  <SubTree ID="Wall_follower"/>                                                                            	
                                  <SubTree ID="Rewind_Handler"/> 
                                </Parallel> 
                              </Sequence>   
                            </BehaviorTree>

                            <BehaviorTree ID="Wall_follower">
                              <Repeat num_cycles="1000">
                                <Sequence name="Wall_follower_seq">
                                  <Sequence name="Wall_follower_basic_seq">	
                                    <Find_Wall name="Find wall"/>	
                                    <Side_Choice name="Side choice"/>                              
                                    <Align name="Align"/>
                                    <Follow_Wall name="Follow a wall"/> 
                                  </Sequence>
                                  <SubTree ID="Corner_Handler"/>                              
                                </Sequence>
                              </Repeat>  
                            </BehaviorTree> 

                            <BehaviorTree ID="Corner_Handler">
                              <ForceSuccess>
                                <Sequence name="Corner_Handler_Seq">
                                  <Side_Empty name="Is the side empty?"/>
                                  <Follow_Corner name="Follow a corner"/> 
                                </Sequence>
                              </ForceSuccess>  
                            </BehaviorTree> 

                            <BehaviorTree ID="Rewind_Handler">
                              <Repeat num_cycles="1000">
                                <Sequence name="Rewind_Handler_Seq">
                                  <Wait_Key name="Wait since a key is pressed"/>
                                  <Rewind name="Rewind"/>
                                </Sequence>
                              </Repeat>  
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
