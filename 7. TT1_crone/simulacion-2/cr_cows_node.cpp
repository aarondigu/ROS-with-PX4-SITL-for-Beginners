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
// cows coordinates in ENU
float cows[][3] =   {{25, 2, 1.2},
                     {17, 9, 1.3},
                     {7, 15, 1.5},
                     {24, 9, 0.6},
                     {23, 18.3, 1.1},
                     {1.5, 3.4, 1.4},
                     {12.2, -2.4, 1.0},
                     {-1.5, 10.2, 0.9} };
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
    ros::init(argc, argv, "cr_cows_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //Subscriber to the x topic
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    //Publisher to publish a DEBUG_VECT        
    ros::Publisher debug_pub = nh.advertise<mavros_msgs::DebugValue>
            ("mavros/debug_value/send", 20);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //To store the vector x,y,z position of the cow
    mavros_msgs::DebugValue debb;
    debb.header.stamp = ros::Time::now();
    debb.header.frame_id = "1"; //ID of the UAV
    debb.name = "";
    debb.type = 1;  //type DEBUG_VECT
    //debb.index = 1;
    debb.data = {0.0, 0.0, 0.0} ;
    //buffer to convert float to sring
    char buffer [9];

    //Number of cows
    int size = sizeof(cows)/sizeof(cows[0]);

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        //If armed and MISSION mode
        if (current_state.mode == "AUTO.MISSION" && current_state.armed ){
            //  check for cows every second
            if (ros::Time::now() - last_request > ros::Duration(1.0)){

                x = current_pose.pose.position.x;
                y = current_pose.pose.position.y;
                z = current_pose.pose.position.z;

                for (int i = 0; i < size; i++){

                    float xc = cows[i][0];
                    float yc = cows[i][1];
                    float zc = cows[i][2];

                    dis = pow((xc - x),2) + pow((yc - y),2) + pow((zc - z),2);
                    dis = sqrtf(dis);

                    // if the drone is less tan 10 meters far from the cow
                    if (dis < 10){
                        sprintf(buffer, "vaca %d", i);
                        ROS_INFO("Found %s", buffer);
                        debb.name = buffer;
                        debb.data[0] = xc;
                        debb.data[1] = yc;
                        debb.data[2] = zc;
                    }

                }

                last_request = ros::Time::now();
            }
        }
        else {
            debb.name = "";
            debb.data[0] = 0.0;
            debb.data[1] = 0.0;
            debb.data[2] = 0.0;
        }
        
        debug_pub.publish(debb);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}