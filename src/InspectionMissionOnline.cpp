#include <second_mission/InspectionMissionOnline.h>

InspectionMissionOnline::InspectionMissionOnline()
{
    // Local variables to listen base pose rispect to the map frame
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Variables to allow the robot the navigation to the first goal 
    navgoal_num = 0;
    moving = false;
    cnt = 0;
    row_id = 0;
    column_id = 0;

    // Define the services for each camera to save images and videos
    image_saver_depth_front = nh.serviceClient<std_srvs::Empty>("/image_saver_front/save");
    image_saver_depth_rear = nh.serviceClient<std_srvs::Empty>("/image_saver_rear/save");
    image_saver_depth_right = nh.serviceClient<std_srvs::Empty>("/image_saver_right/save");
    image_saver_depth_left = nh.serviceClient<std_srvs::Empty>("/image_saver_left/save");

    // Setting the gird dimention by shell, to avoid this option just write down the rows ad columns value
    cout << "Set the ispection area" << endl;
    cout << "Number of rows: " << endl;
    cin >> rows;
    cout << "Number of columns: " << endl;
    cin >> columns;

    // Getting and then setting the whole ros params related to the navigation goals
    nh.getParam("/environment/objects", environment_list);

    environment_list_push = environment_list[0];
    auto a = environment_list[0];
    std::vector<decltype(a)> v;
    // cout << "v appena inizializzato" << endl; 
    for (int i = 0; i < environment_list.size(); i++)
    {
            v.push_back(environment_list[i]);
            cout << v[i] << endl;
    }
    
    // Record transform value (rot and trasl) between odom and map
    try{
        tfBuffer.canTransform("map", "base", ros::Time(0), ros::Duration(3.0));
        transformStamped = tfBuffer.lookupTransform("map", "base", ros::Time(0));
    }
    catch (tf2::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    Eigen::Vector3d goal_position_base(1.0, 0.0, 0.0);
    base_to_map = tf2::transformToEigen(transformStamped);

    goal_position_map = base_to_map*goal_position_base;

    environment_list_push["label"] = "IspectionNavigationGoal" + to_string(row_id)+ to_string(column_id);
    environment_list_push["name"] = "IspectionNavigationGoal" + to_string(row_id)+ to_string(column_id);
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

    // cout << "v dopo aver messo il primo punto" << endl;
    // TODO occhiio forse serve 1 non zero per non perdere il primo elemento che c'è in memoria nel ros param, 
    // FORSE COSI DOPO SOVRASCRIVIAMO AI PRIMI ELEMENNTI
    for (int i = 0; i < v.size(); i++)
    {
            environment_list2[i] = v[i];
            cout << v[i] << endl;
    }

    nh.setParam("/environment/objects", environment_list2);

    navigation_goal_pub = nh.advertise<state_machine_msgs::StateWithInitialState>("/behavior_engine/execute_state/start_state", 1000);

    // Initialize and control the subscriber
    sub_travel = nh.subscribe(path_planning_topic, 1, &InspectionMissionOnline::readCallback, this); 
    
    first_state = ros::topic::waitForMessage<navigation_manager_msgs::LocalPlannerStatus>(this->path_planning_topic, ros::Duration(1.0));
    if (this->first_state == NULL) {
        ROS_ERROR("Cannot contact the planner robot states topic! This is not safe anymore.");
    }

    // row_id ++;
    // cout << "nel costruttore, alla fine, la row_id è pari a " << row_id << endl;

    // cout << "nel costruttore, alla fine, la nav_goal è pari a " << navgoal_num << endl;

    // cout << "nel costruttore, alla fine, la name.at(nav_goal_num) è pari a " << name.at(navgoal_num) << endl;

    // publish_navgoal(name.at(navgoal_num));

}

InspectionMissionOnline::~InspectionMissionOnline()
{
    // Empty destructor
}

void InspectionMissionOnline::readCallback(const navigation_manager_msgs::LocalPlannerStatus & msg)
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

void InspectionMissionOnline::updating_navgoal(int row_id, int column_id, int navgoal_num)
{
    // METTERCI UN BOOL PER IL CHECK AL MASSIMO 
    if (row_id == 0 && column_id == 0)
    {
        // Non fare nulla
    }
    else
    {

    // cout << "ENTRATO NELLA UPDATING "  << endl;

    // Local variables to listen base pose rispect to the map frame
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    nh.getParam("/environment/objects", environment_list);

    environment_list_push = environment_list[0];
    auto a = environment_list[0];
    std::vector<decltype(a)> v;
    cout << "v dopo aver ri gettato i parametri" << endl;
    for (int i = 0; i < environment_list.size(); i++)
    {
            v.push_back(environment_list[i]);
            cout << v[i] << endl;
    }
    // Record transform value (rot and trasl) between odom and map
    try{
        tfBuffer.canTransform("map", "base", ros::Time(0), ros::Duration(3.0));
        transformStamped = tfBuffer.lookupTransform("map", "base", ros::Time(0));
    }
    catch (tf2::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    // cout << "prima volto che entro nel updating nav goal funzione, la row_id è pari a " << row_id << endl;

    // cout << "prima volta che entro nel updating nav goal funzione, la nav_goal è pari a " << navgoal_num << endl;
    Eigen::Vector3d goal_position_base(1.0, 0.0, 0.0);
    base_to_map = tf2::transformToEigen(transformStamped);

    goal_position_map = base_to_map*goal_position_base;

    environment_list_push["label"] = "IspectionNavigationGoal" + to_string(row_id)+ to_string(column_id);
    environment_list_push["name"] = "IspectionNavigationGoal" + to_string(row_id)+ to_string(column_id);
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

    // cout << "v dopo aver ri gettato i parametri e anche settato il nuovo punto" << endl;
    for (int i = 0; i < v.size(); i++)
    {
            environment_list2[i] = v[i];
            cout << v[i] << endl;
    }

    // TODO TROVA IL MODO DI FARGLI SETTARE QUESTI A PARTIRE DAI PRECEDENTI, MAGARI POST GET FAI UN CICLO CHE PARTE DA NAV GOAL TIPO

    // From the vector to the XmlRpcValue again to be aligned with the rosparam type
    // environment_list2[navgoal_num] = v[navgoal_num];
    // ros::Duration(2).sleep();
    nh.setParam("/environment/objects", environment_list2);

    // navgoal_num ++;
    // cout << "prima di uscire, dopo aver fatto navgoal_num ++, la nav_goal è pari a " << navgoal_num << endl;
    // cout << "prima di uscire, dopo aver fatto navgoal_num ++, la name.at(nav_goal_num) è pari a " << name.at(navgoal_num) << endl;
    
    }
}

void InspectionMissionOnline::publish_navgoal(std::string name)
{
    navigation_goal_msg.state.name = "GoTo" + name;
    navigation_goal_msg.state.type = "navigation_behavior_plugins::ReactiveNavigation";
    navigation_goal_msg.state.settings = "- name: navigation_goal\n  type: NavigationGoal\n  value: " + name;
    vector<string> empty_string_vect = {};
    navigation_goal_msg.initial_state.names = empty_string_vect;
    navigation_goal_pub.publish(navigation_goal_msg);
    // ros::Duration(2).sleep();
}

void InspectionMissionOnline::run()
{
    
    if(!moving && row_id <= rows ) // && column_id < columns
    {
        updating_navgoal(row_id, column_id, navgoal_num);
        publish_navgoal(name.at(navgoal_num));
        moving = true;
        row_id ++;
        navgoal_num ++;
        // column_id ++;
    }

}
