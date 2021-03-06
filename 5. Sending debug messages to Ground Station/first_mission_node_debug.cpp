/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/DebugValue.h>
#include <math.h>  
#include <stdio.h>


const float  PI = 3.14159265358979f;
//pose: x y x, Orientation: x y z w
float mission[][5] =   {{0, 0, 3, 0},
					   {5, 0, 3, 0},
					   {5, 5, 3, 0.5*PI},
					   {0, 5, 3, PI},
					   {0, 0, 3, 1.5*PI}};

//To store angle of rotation around z
float angle;

//To store current pose
float x,y,z;
//Distance between desired pose and current pose
float dis = 0;



mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_node_debug");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    //Subscriber to the pose topic
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    //Service to land drone
    ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    //Publisher to publish a DEBUG_VECT        
    ros::Publisher debug_pub = nh.advertise<mavros_msgs::DebugValue>
            ("mavros/debug_value/send", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    int point_counter = 0;

    int mission_size = sizeof(mission) / sizeof(mission[0]);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = mission[point_counter][0];
    pose.pose.position.y = mission[point_counter][1];
    pose.pose.position.z = mission[point_counter][2];
    angle = mission[point_counter][3];
    pose.pose.orientation.w = cos(angle/2);
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = sin(angle/2);

    //To store the vector x,y,z position of the cow
    mavros_msgs::DebugValue debb;
    //debb.header.seq = 100;
    debb.header.stamp = ros::Time::now();
    debb.header.frame_id = "1"; //ID of the UAV
    debb.name = "";
    debb.type = 1;  //type DEBUG_VECT
    //debb.index = 1;
    debb.data = {0.0, 0.0, 0.0} ;

    //buffer to convert float to string
    char buffer [9];


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.min_pitch = 0.0f;
    land_cmd.request.yaw = 0.0f;
    land_cmd.request.latitude = 0.0f;
    land_cmd.request.longitude = 0.0f;
    land_cmd.request.altitude = 0.0f;

    ros::Time last_request = ros::Time::now();
    ros::Time last_request_2 = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        //If armed and offboard mode, check for next setpoint
        if (current_state.mode == "OFFBOARD" && current_state.armed){
        	x = current_pose.pose.position.x;
        	y = current_pose.pose.position.y;
        	z = current_pose.pose.position.z;

        	dis = pow((pose.pose.position.x - x),2) + pow((pose.pose.position.y - y),2) + pow((pose.pose.position.z - z),2);
        	dis = sqrtf(dis);

        	if (dis <= 0.25){ //if distance to the target is <=0.25 meters
        		if (point_counter >= mission_size){
        			pose.pose.position.x = 0;    				
                    pose.pose.position.y = 0;
    				pose.pose.position.z = 0;


                    if( landing_client.call(land_cmd) && land_cmd.response.success){
                        ROS_INFO("Vehicle disarmed");
                        break;
                    }

        		}
        		else{
                    ROS_INFO("Reached target %d out of %d", point_counter + 1, mission_size);
	        		point_counter++;
	        		pose.pose.position.x = mission[point_counter][0];
	    			pose.pose.position.y = mission[point_counter][1];
	    			pose.pose.position.z = mission[point_counter][2];
                    angle = mission[point_counter][3];
                    pose.pose.orientation.w = cos(angle/2);
                    pose.pose.orientation.x = 0;
                    pose.pose.orientation.y = 0;
                    pose.pose.orientation.z = sin(angle/2);

                    
                    sprintf(buffer, "target %d", point_counter);
                    debb.name = buffer;
                    debb.data[0] = x;
                    debb.data[1] = y;
                    debb.data[2] = z;
    			}
        	}

        }
        

        local_pos_pub.publish(pose);
        debug_pub.publish(debb); //publishing DEBUG_VECT

        ros::spinOnce();
        rate.sleep();
        
    }

    return 0;
}