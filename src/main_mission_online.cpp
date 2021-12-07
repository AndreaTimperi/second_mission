#include <second_mission/InspectionMissionOnline.h>

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "second_mission_online_node");
  ros::NodeHandle nh;
  ros::Rate r(400);

  InspectionMissionOnline InspectionMissionOnlineObj;
  
  while (ros::ok()) 
  {
    
    InspectionMissionOnlineObj.run();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
