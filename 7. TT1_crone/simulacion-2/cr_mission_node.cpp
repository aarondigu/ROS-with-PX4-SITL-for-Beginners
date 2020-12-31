// MPC_XY_CRUISE = 3.0, maximum horizontal velocity of 3 m/s
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointReached.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

mavros_msgs::WaypointReached wp_reached;
void current_wp(const mavros_msgs::WaypointReached::ConstPtr& msg){
    wp_reached = *msg;
    ROS_INFO("REACHED WAYPOINT %d", wp_reached.wp_seq);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cr_mission_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber waypoint_reached_sub = nh.subscribe<mavros_msgs::WaypointReached>
            ("mavros/mission/reached", 10, current_wp);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient waypoint_pull_client = nh.serviceClient<mavros_msgs::WaypointPull>
            ("mavros/mission/pull");
    ros::ServiceClient waypoint_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>
            ("mavros/mission/clear");
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //clear mission on fcu
    mavros_msgs::WaypointClear wpp_clear;
    while(ros::ok() && waypoint_clear_client.call(wpp_clear) && !wpp_clear.response.success) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Mission cleared");

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "AUTO.MISSION";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::WaypointPull wpp;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
            // if not armed and waypoint list > 0
            if( ros::Time::now() - last_request > ros::Duration(5.0)) {
                if( !current_state.armed && waypoint_pull_client.call(wpp) && wpp.response.wp_received > 0){
                    ROS_INFO("Received %d waypoints", wpp.response.wp_received);
                    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                        ROS_INFO("Auto enabled");
                        if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                            ROS_INFO("Vehicle armed");
                        } 
                    }
            }
            last_request = ros::Time::now();
            }
            //if not armed and last waypoint reached
            if (!current_state.armed && (wp_reached.wp_seq + 1 == wpp.response.wp_received)){
                if (waypoint_clear_client.call(wpp_clear) && wpp_clear.response.success) {
                        ROS_INFO("Mission cleared and drone landed correctly!!!");
                        wp_reached.wp_seq = 0;
                }
            }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}