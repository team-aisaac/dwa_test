#include<ros/ros.h>
#include<dwa_test/dwa_planner.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nodeHandle("~");
    DWAPlanner dwa;

    while(nodeHandle.ok()){
        ros::spin();
    }
    //ros::AsyncSpinner spinner(1);
    //spinner.start();

    //ros::Rate rate(60);
    //while(nodeHandle.ok()){
    //    rate.sleep();
    //}

    //spinner.stop();

    ros::waitForShutdown();
    return 0;
}
