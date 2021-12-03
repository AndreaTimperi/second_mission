#include <second_mission/InspectionMission.h>

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "second_mission_node");
  ros::NodeHandle nh;
  ros::Rate r(400);

  InspectionMission InspectionMissionObj(nh);
  
  while (ros::ok()) 
  {
    
    InspectionMissionObj.run();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
