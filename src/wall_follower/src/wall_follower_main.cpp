#include "actions.h"
#include "wall_follower.h"

//using namespace BT;

/** Behavior Tree are used to create a logic to decide what
 * to "do" and when. For this reason, our main building blocks are
 * Actions and Conditions.
 */



const char *xml_tree = R"(
                        <root main_tree_to_execute="Main_Tree">
                            <BehaviorTree ID="Main_Tree"> 
                              <ReactiveFallback> 

                                <Sequence name ="Key_Seq">
                                  <Key_Detector name="Is key detected?"/>
                                  <SubTree ID="Rewind_Subtree"/>
                                </Sequence>
                                
                                <SubTree ID="Wall_Follower_Subtree"/>
                                
                              </ReactiveFallback>
                            </BehaviorTree>

                            <BehaviorTree ID="Wall_Follower_Subtree">
                                <Sequence name="Wall_Follower_Seq">
                                  <Find_Wall name="Find wall"/>	
                                  <Side_Choice name="Side choice"/>                              
                                  <Align name="Align"/>
                                  <Follow_Wall name="Follow a wall"/> 
                                  <Follow_Corner name="Follow a corner"/>                               
                                </Sequence> 
                            </BehaviorTree> 

                            <BehaviorTree ID="Rewind_Subtree">
                                <Sequence name ="Rewind_Seq">
                                  <Turn name="Turn"/> 
                                  <Rewind name="Rewind"/>
                                </Sequence> 
                            </BehaviorTree>

                        </root>
                        )"; 



/*
const char *xml_tree = R"(
                        <root main_tree_to_execute="Main_Tree">
                            <BehaviorTree ID="Main_Tree"> 
                              <ReactiveFallback> 

                                <Sequence name ="Collision_Seq">
                                  <Collision_Detector name="Is collision detected?"/> 
                                  <Turn name="Turn"/>
                                </Sequence>

                                <Sequence name ="Key_Seq">
                                  <Key_Detector name="Is key detected?"/>
                                  <SubTree ID="Rewind_Subtree"/>
                                </Sequence>
                                
                                <SubTree ID="Wall_Follower_Subtree"/>
                                
                              </ReactiveFallback>
                            </BehaviorTree>

                            <BehaviorTree ID="Wall_Follower_Subtree">
                                <Sequence name="Wall_Follower_Seq">
                                  <Find_Wall name="Find wall"/>	
                                  <Side_Choice name="Side choice"/>                              
                                  <Align name="Align"/>
                                  <Follow_Wall name="Follow a wall"/> 
                                  <Follow_Corner name="Follow a corner"/>                               
                                </Sequence> 
                            </BehaviorTree> 

                            <BehaviorTree ID="Rewind_Subtree">
                                <Sequence name ="Rewind_Seq">
                                  <Turn name="Turn"/> 
                                  <Rewind name="Rewind"/>
                                </Sequence> 
                            </BehaviorTree>

                        </root>
                        )";    
*/

/*

//THIS WORKS WITH INVERTED CONDITIONS
const char *xml_tree = R"(
                        <root main_tree_to_execute="Main_Tree">
                            <BehaviorTree ID="Main_Tree"> 
                              <Fallback> 

                                <ReactiveSequence name ="Wall_Follower_Seq">
                                  <Collision_Detector name="Is collision NOT detected?"/> 
                                  <Key_Detector name="Is key NOT detected?"/>
                                  <SubTree ID="Wall_Follower_Subtree"/>
                                </ReactiveSequence>
                                
                                <SubTree ID="Rewind_Subtree"/>

                              </Fallback>
                            </BehaviorTree>

                            <BehaviorTree ID="Wall_Follower_Subtree">
                                <Sequence name="Wall_Follower_Seq">
                                  <Find_Wall name="Find wall"/>	
                                  <Side_Choice name="Side choice"/>                              
                                  <Align name="Align"/>
                                  <Follow_Wall name="Follow a wall"/> 
                                  <Follow_Corner name="Follow a corner"/>                               
                                </Sequence> 
                            </BehaviorTree> 

                            <BehaviorTree ID="Rewind_Subtree">
                               <Fallback>
                                <ReactiveSequence name ="Wall_Follower_Seq">
                                  <Collision_Detector name="Is collision NOT detected?"/> 
                                  <Sequence name ="Rewind_Seq">
                                    <Turn name="Turn"/> 
                                    <Rewind name="Rewind"/>
                                  </Sequence>
                                </ReactiveSequence>

                                <Turn name="Turn"/>
                              </Fallback>  
                            </BehaviorTree>

                        </root>
                        )"; 
*/
   

/*
  <BehaviorTree ID="Corner_Handler">
    <Fallback name="Corner_Handler_Seq">
      <Side_Occupied name="Is the side Occupied?"/>
      <Follow_Corner name="Follow a corner"/> 
    </Fallback>  
  </BehaviorTree>
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