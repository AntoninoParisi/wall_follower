#pragma once
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
                              <ReactiveFallback name ="Collision_Fall">

                                <Sequence name ="Collision_Seq">
                                  <Collision_Detector name="Is collision detected?"/>
                                  <SubTree ID="Safety_Subtree"/>
                                </Sequence>

                                <ReactiveFallback name ="Key_Fall"> 

                                  <Sequence name ="Key_Seq">
                                    <Key_Detector name="Is key detected?"/>
                                    <SubTree ID="Rewind_Subtree"/>
                                  </Sequence>
                                  
                                  <SubTree ID="Wall_Follower_Subtree"/>
                                  
                                </ReactiveFallback>
                              
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
                                  <Turn name="Turn" angle="180" time="10" direction="clockwise"/>
                                  <Rewind name="Rewind"/>
                                </Sequence> 
                            </BehaviorTree>

                            <BehaviorTree ID="Safety_Subtree">
                              <Sequence name ="Safety_Seq">
                                <Turn name="Turn" angle="90" time="2" direction="anticlockwise"/>
                              </Sequence> 
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