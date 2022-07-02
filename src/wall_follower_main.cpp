#include "actions.h"

using namespace std;
using namespace BT;


  

/** Behavior Tree are used to create a logic to decide what
 * to "do" and when. For this reason, our main building blocks are
 * Actions and Conditions.
 */


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
 
  rclcpp::spin(std::make_shared<Wall_Follower>());

  cout << "fine " << endl;
  
  return 0;
}
