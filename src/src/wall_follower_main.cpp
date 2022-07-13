#pragma once
#include "actions.h"
#include "wall_follower.h"

//using namespace BT;

/** Behavior Tree are used to create a logic to decide what
 * to "do" and when. For this reason, our main building blocks are
 * Actions and Conditions.
 */

// collision non blocca rewind

const char *xml_tree = R"(
                        <root main_tree_to_execute="Main_Tree">
                            <BehaviorTree ID="Main_Tree"> 
                              <ReactiveFallback name ="Collision_Fall">

                                <Sequence name ="Collision_Seq">
                                  <Collision_Detector name="Is collision detected?"/>
                                  <SubTree ID="Safety_Subtree"/>
                                </Sequence>

                                <ReactiveFallback name ="Key_Fall"> 

                                  <SequenceStar name ="Key_Seq">
                                    <Key_Detector name="Is key detected?"/>
                                    <SubTree ID="Rewind_Subtree"/>
                                  </SequenceStar>
                                  
                                  <SubTree ID="Wall_Follow_Subtree"/>
                                  
                                </ReactiveFallback>
                              
                              </ReactiveFallback>
                            </BehaviorTree>


                            <BehaviorTree ID="Wall_Follow_Subtree">
                                <Sequence name="Wall_Follow_Seq">
                                  <Find_Wall name="Find wall"/>	
                                  <Side_Choice name="Side choice"/>                              
                                  <Align name="Align"/>
                                  <Follow_Wall name="Follow a wall"/> 
                                  <Follow_Corner name="Follow a corner"/>                               
                                </Sequence> 
                            </BehaviorTree> 

                            <BehaviorTree ID="Rewind_Subtree">
                                <SequenceStar name ="Rewind_Seq">
                                  <Set_Save name="Set_Safe" mode="off"/>
                                  <Turn name="Turn" angle="180" time="2" direction="clockwise"/>
                                  <Rewind name="Rewind"/>
                                  <Set_Save name="Set_Safe" mode="on"/>
                                </SequenceStar> 
                            </BehaviorTree>

                            <BehaviorTree ID="Safety_Subtree">
                              <Sequence name ="Safety_Seq">
                                <Go_Back name="Go_Back" distance="0.5" time="2"/>
                              </Sequence> 
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