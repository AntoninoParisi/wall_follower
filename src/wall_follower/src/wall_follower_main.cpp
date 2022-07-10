#include "actions.h"
#include "wall_follower.h"

//using namespace BT;

/** Behavior Tree are used to create a logic to decide what
 * to "do" and when. For this reason, our main building blocks are
 * Actions and Conditions.
 */

/*

// WhileDoElse VERSION (non pare dare vantaggi, e non è istantaneo)

const char *xml_tree = R"(
                        <root main_tree_to_execute="MainTree">
                            <BehaviorTree ID="MainTree">                                                                    
                              <WhileDoElse>
                                <Key_Detector name="Is collision NOT detected?"/>
                                <SubTree ID="Wall_Follower"/>
                                <Sequence name ="Rewind_Seq">
                                  <Turn name="Turn"/> 
                                  <Rewind name="Rewind"/>
                                </Sequence>
                              </WhileDoElse>                                                                               	                                                         
                            </BehaviorTree>

                            <BehaviorTree ID="Wall_Follower">
                                <Sequence name="Wall_Follower_Seq">
                                  <Find_Wall name="Find wall"/>	
                                  <Side_Choice name="Side choice"/>                              
                                  <Align name="Align"/>
                                  <Follow_Wall name="Follow a wall"/> 
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
*/

const char *xml_tree = R"(
                        <root main_tree_to_execute="MainTree">
                            <BehaviorTree ID="MainTree">                                                                    
                              <Parallel success_threshold="3">
                                <SubTree ID="Key_Handler"/>
                                <SubTree ID="Collision_Handler"/>
                                <SubTree ID="Wall_Follower"/>
                              </Parallel>                                                                               	                                                         
                            </BehaviorTree>

                            <BehaviorTree ID="Wall_Follower">
                                <Sequence name="Wall_Follower_Seq">
                                  <Find_Wall name="Find wall"/>	
                                  <Side_Choice name="Side choice"/>                              
                                  <Align name="Align"/>
                                  <Follow_Wall name="Follow a wall"/> 
                                  <SubTree ID="Corner_Handler"/>                               
                                </Sequence> 
                            </BehaviorTree> 

                            <BehaviorTree ID="Corner_Handler">
                              <Fallback name="Corner_Handler_Seq">
                                <Side_Occupied name="Is the side Occupied?"/>
                                <Follow_Corner name="Follow a corner"/> 
                              </Fallback>  
                            </BehaviorTree> 

                            <BehaviorTree ID="Collision_Handler">
                              <Fallback name="Collision_Handler_Seq">
                                <Collision_Detector name="Is collision NOT detected?"/> 
                                <Turn name="Turn"/> 
                              </Fallback>  
                            </BehaviorTree> 

                            <BehaviorTree ID="Key_Handler">
                              <Fallback name="Key_Handler_Seq">
                                <Key_Detector name="Is collision NOT detected?"/> 
                                <Sequence name ="Rewind_Seq">
                                  <Turn name="Turn"/> 
                                  <Rewind name="Rewind"/>
                                </Sequence>  
                              </Fallback>  
                            </BehaviorTree>

                        </root>
                        )";    

/*
const char *xml_tree = R"(
                        <root main_tree_to_execute="MainTree">
                            <BehaviorTree ID="MainTree">                                                                    
                              <Fallback name="Main_Sequence">
                                <SubTree ID="Primary_Task"/>
                                <SubTree ID="Secondary_Task"/>
                              </Fallback>                                                                               	                                                         
                            </BehaviorTree>

                            <BehaviorTree ID="Primary_Task">
                              <Sequence>                                     
                                <Wait_Key name="Is key NOT pressed?"/>  
                                <Collision_Detect name="Is collision NOT detected?"/>                                    
                                <SubTree ID="Wall_Follower"/>
                              </Sequence>
                            </BehaviorTree>

                            <BehaviorTree ID="Wall_Follower">
                                <Sequence name="Wall_Follower_Seq">
                                  <Find_Wall name="Find wall"/>	
                                  <Side_Choice name="Side choice"/>                              
                                  <Align name="Align"/>
                                  <Follow_Wall name="Follow a wall"/> 
                                  <SubTree ID="Corner_Handler"/>                               
                                </Sequence> 
                            </BehaviorTree> 

                            <BehaviorTree ID="Corner_Handler">
                              <Fallback name="Corner_Handler_Seq">
                                <Side_Occupied name="Is the side Occupied?"/>
                                <Follow_Corner name="Follow a corner"/> 
                              </Fallback>  
                            </BehaviorTree> 

                            <BehaviorTree ID="Secondary_Task">
                              <Sequence name="Rewind_Sequence_Seq">
                                <Turn name="Turn"/>
                                <Rewind name="Rewind"/> 
                              </Sequence>  
                            </BehaviorTree>

                        </root>
                        )";    
*/   


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