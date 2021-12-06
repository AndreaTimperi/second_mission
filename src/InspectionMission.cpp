#include <second_mission/InspectionMission.h>

InspectionMission::InspectionMission(ros::NodeHandle nh)
{
    // Local variables to listen base pose rispect to the map frame
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Variables to allow the robot the navigation to the first goal 
    navgoal_num = 0;
    moving = false;
    cnt = 0;

    // Define the services for each camera to save images and videos
    image_saver_depth_front = nh.serviceClient<std_srvs::Empty>("/image_saver_front/save");
    image_saver_depth_rear = nh.serviceClient<std_srvs::Empty>("/image_saver_rear/save");
    image_saver_depth_right = nh.serviceClient<std_srvs::Empty>("/image_saver_right/save");
    image_saver_depth_left = nh.serviceClient<std_srvs::Empty>("/image_saver_left/save");

    // Getting and then setting the whole ros params related to the navigation goals
    nh.getParam("/environment/objects", environment_list);

    environment_list_push = environment_list[0];
    auto a = environment_list[0];
    std::vector<decltype(a)> v;
    for (int i = 0; i < environment_list.size(); i++)
    {
            v.push_back(environment_list[i]);
    }

    // IDEA DI FARE UN CICLO FOR, settiamo in f(x) dell'area da esplorare, un Param della granularità, aggiorna le pos in modo ciclico
    
    // Setting the gird dimention by shell, to avoid this option just write down the rows ad columns value
    cout << "Set the ispection area" << endl;
    cout << "Number of rows: " << endl;
    cin >> rows;
    cout << "Number of columns: " << endl;
    cin >> columns;

    for (int j = 0; j < columns; ++j)
    {
        for (int i = 0; i < rows; ++i)
        { 
            // Record transform value (rot and trasl) between odom and map
            try{
                tfBuffer.canTransform("map", "base", ros::Time(0), ros::Duration(3.0));
                transformStamped = tfBuffer.lookupTransform("map", "base", ros::Time(0));
            }
            catch (tf2::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

            Eigen::Vector3d goal_position_base(1.0 + i, 0.0 + j, 0.0);
            base_to_map = tf2::transformToEigen(transformStamped);

            goal_position_map = base_to_map*goal_position_base;

            environment_list_push["label"] = "IspectionNavigationGoal" + to_string(i)+ to_string(j);
            environment_list_push["name"] = "IspectionNavigationGoal" + to_string(i)+ to_string(j);
            environment_list_push["pose"]["header"]["frame_id"] = "map";
            environment_list_push["pose"]["pose"]["orientation"]["w"] = transformStamped.transform.rotation.w;
            environment_list_push["pose"]["pose"]["orientation"]["x"] = transformStamped.transform.rotation.x;
            environment_list_push["pose"]["pose"]["orientation"]["y"] = transformStamped.transform.rotation.y;
            environment_list_push["pose"]["pose"]["orientation"]["z"] = transformStamped.transform.rotation.z;
            environment_list_push["pose"]["pose"]["position"]["x"] = goal_position_map(0);
            environment_list_push["pose"]["pose"]["position"]["y"] = goal_position_map(1);
            environment_list_push["pose"]["pose"]["position"]["z"] = goal_position_map(2); // TODO controllare questo perchè se ci sono pendenza vada bene, se lavori step by step its fine
            environment_list_push["tolerance"]["rotation"] = 0.1;
            environment_list_push["tolerance"]["translation"] = 0.05;
            environment_list_push["type"]= "navigation_goal";
            
            v.push_back(environment_list_push);


            name.push_back(environment_list_push["label"]);
        }
    }

    // From the vector to the XmlRpcValue again to be aligned with the rosparam type
    for (int i = 0; i < v.size(); i++)
    {
            environment_list2[i] = v[i];
    }

    nh.setParam("/environment/objects", environment_list2);

    navigation_goal_pub = nh.advertise<state_machine_msgs::StateWithInitialState>("/behavior_engine/execute_state/start_state", 1000);

    // Initialize and control the subscriber
    sub_travel = nh.subscribe(path_planning_topic, 1, &InspectionMission::readCallback, this); 
    first_state = ros::topic::waitForMessage<navigation_manager_msgs::LocalPlannerStatus>(this->path_planning_topic, ros::Duration(1.0));
    if (this->first_state == NULL) {
        ROS_ERROR("Cannot contact the planner robot states topic! This is not safe anymore.");
    }

    ros::Duration(0.5).sleep();
}

InspectionMission::~InspectionMission()
{
    // Empty destructor
}

void InspectionMission::readCallback(const navigation_manager_msgs::LocalPlannerStatus & msg)
{
    if(msg.status == 1 || cnt == 0)
    {
        // ROS_INFO("Moving to the GOAL");
        moving = true;
        cnt ++;
    }
    else
    {   
        moving = false;
        ROS_INFO("Arrived");
        ros::Duration(1.5).sleep(); 
        // It works if an "image_view image_saver" node is running with the camera_path and the png file name declared
        if(image_saver_depth_front.call(srv_click_front))
        if(image_saver_depth_rear.call(srv_click_rear))
        if(image_saver_depth_right.call(srv_click_right))
        if(image_saver_depth_left.call(srv_click_left))
        ros::Duration(1.5).sleep();
            ROS_INFO("Image taken");
            
    }
}

void InspectionMission::publish_navgoal(std::string name)
{
    navigation_goal_msg.state.name = "GoTo" + name;
    navigation_goal_msg.state.type = "navigation_behavior_plugins::ReactiveNavigation";
    navigation_goal_msg.state.settings = "- name: navigation_goal\n  type: NavigationGoal\n  value: " + name;
    vector<string> empty_string_vect = {};
    navigation_goal_msg.initial_state.names = empty_string_vect;
    navigation_goal_pub.publish(navigation_goal_msg);
}

void InspectionMission::run()
{

    if(!moving && navgoal_num < name.size())
    {
        publish_navgoal(name.at(navgoal_num));
        moving = true;
        navgoal_num ++;
    }

}