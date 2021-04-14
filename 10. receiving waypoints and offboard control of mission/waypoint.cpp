// MPC_XY_CRUISE = 3.0, maximum horizontal velocity of 3 m/s
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/PositionTarget.h>
#include <math.h>  
#include <stdio.h>

using namespace std; 
const float  PI = 3.14159265358979f;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

mavros_msgs::WaypointReached wp_reached;
void current_wp(const mavros_msgs::WaypointReached::ConstPtr& msg){
    wp_reached = *msg;
    ROS_INFO("REACHED WAYPOINT %d", wp_reached.wp_seq);
}


vector<mavros_msgs::Waypoint> waypoints;
void waypoints_wp(const mavros_msgs::WaypointList msg){
  if (msg.waypoints.size() > 0)
    waypoints = msg.waypoints;
}


geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber waypoint_reached_sub = nh.subscribe<mavros_msgs::WaypointReached>
            ("mavros/mission/reached", 10, current_wp);

    ros::Subscriber waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 10, waypoints_wp);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient waypoint_pull_client = nh.serviceClient<mavros_msgs::WaypointPull>
            ("mavros/mission/pull");
    ros::ServiceClient waypoint_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>
            ("mavros/mission/clear");

    // Publica en set_position/local
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // Publica en setpoint_raw/local
    ros::Publisher local_raw_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);

    // Publica los wp goal
    ros::Publisher next_wp_pub = nh.advertise<geometry_msgs::Point>
            ("next_goal_wp", 20);

    // Suscriptor de la pose local
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);

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
    offb_set_mode.request.custom_mode = "AUTO.LAND";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;




    float wp_x = 0.0;
    float wp_y = 0.0;
    float wp_z = 0.0;

    // Mientras no haya waypoints
    while(ros::ok() && waypoints.size() == 0){
        ros::spinOnce();
        rate.sleep();
    }

    // Imprimir waypoints
    for(int i = 0; i < waypoints.size(); i++)
      {
        wp_x = waypoints[i].x_lat;
        wp_y =  waypoints[i].y_long;
        wp_z =  waypoints[i].z_alt;
        ROS_INFO("Waypoint %d @ %f, %f, %f", i, wp_x, wp_y, wp_z);
      }

    // Limpiar de nuevo los waypoints
    while(ros::ok() && waypoint_clear_client.call(wpp_clear) && !wpp_clear.response.success) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Mission cleared again");


    int current_wp = 0; // El waypoint goal actual 
    float dis_wp = 0.0; // Distancia al waypoint goal actual
    float x, y, z; // Guardar la pose actual
    float radius_acc = 0.5; // Radio de aceptación para llegar a un waypoint 
    float vel_max = 3.0; // Magnitud de velocidad maxima
    float radius_freno = 2.5; // A que distancia del waypoint va a empezar a frenar en metros
    int last_wp_reached = 0; // para saber si ya se llego al ultimo waypoint
    float vec_x = 0.0;
    float vec_y = 0.0;
    float vec_z = 0.0;

    geometry_msgs::PoseStamped pose;
    // Inicializa la pose goal
    pose.pose.position.x = waypoints[current_wp].x_lat;
    pose.pose.position.y = waypoints[current_wp].y_long;
    pose.pose.position.z = waypoints[current_wp].z_alt;

    // Inicializa el Point Goal
    geometry_msgs::Point next_wp_goal;
    next_wp_goal.x = waypoints[current_wp].x_lat;
    next_wp_goal.y = waypoints[current_wp].y_long;
    next_wp_goal.z = waypoints[current_wp].z_alt;


    mavros_msgs::PositionTarget pos_tar;
    // Inicial el goal raw
    pos_tar.coordinate_frame =  mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // Masks validas: vx,vy,vz,yaw (3015); vx,vy,z,yaw (3043);
    pos_tar.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                        mavros_msgs::PositionTarget::IGNORE_PY |
                        mavros_msgs::PositionTarget::IGNORE_PZ |
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE ;
    pos_tar.position.x = waypoints[current_wp].x_lat;
    pos_tar.position.y = waypoints[current_wp].y_long;
    pos_tar.position.z = waypoints[current_wp].z_alt;
    pos_tar.velocity.x = 0.0;
    pos_tar.velocity.y = 0.0;
    pos_tar.velocity.z = 0.0;
    pos_tar.yaw = 0;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_raw_pub.publish(pos_tar);
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
        if( current_state.mode == "OFFBOARD" && !current_state.armed){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                
        }


        x = current_pose.pose.position.x;
        y = current_pose.pose.position.y;
        z = current_pose.pose.position.z;

        dis_wp = pow((pos_tar.position.x - x),2) + pow((pos_tar.position.y - y),2) + pow((pos_tar.position.z -  z),2);
        dis_wp = sqrtf(dis_wp);


        if (dis_wp <= radius_acc ){
            ROS_INFO("Waypoint %d reached, out of %d", current_wp, waypoints.size() -1 );

            if (current_wp == waypoints.size() - 1){
                last_wp_reached = 1;
                if( current_state.mode == "OFFBOARD" && set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("lANDING...");
            }
            }
            else
            {
                current_wp++;
                // Setpoint de posición
                pos_tar.position.x = waypoints[current_wp].x_lat;
                pos_tar.position.y = waypoints[current_wp].y_long;
                pos_tar.position.z = waypoints[current_wp].z_alt;
                next_wp_goal.x = waypoints[current_wp].x_lat;
                next_wp_goal.y = waypoints[current_wp].y_long;
                next_wp_goal.z = waypoints[current_wp].z_alt;
            }
        }


        // Determinando la velocidad, si el drone esta fuera del radio de freno, usa la maxima, si está dentro, empieza a frenar 
        if (dis_wp >= radius_freno){
            vec_x = vel_max * (pos_tar.position.x - x) / dis_wp;
            vec_y = vel_max * (pos_tar.position.y - y) / dis_wp;
            vec_z = vel_max * (pos_tar.position.z - z) / dis_wp;
        }
        else{
            // Distancia entre el wp actual y el anterior
            float dis_wp_anterior = pow((pos_tar.position.x - waypoints[current_wp - 1].x_lat),2) + pow((pos_tar.position.y - waypoints[current_wp - 1].y_long),2) + pow((pos_tar.position.z -  waypoints[current_wp - 1].z_alt),2);
            dis_wp_anterior = sqrtf(dis_wp_anterior);
            // La distancia va a disminuir conforme se acerque al punto
            vec_x = vel_max * (pos_tar.position.x - x) / dis_wp_anterior;
            vec_y = vel_max * (pos_tar.position.y - y) / dis_wp_anterior;
            vec_z = vel_max * (pos_tar.position.z - z) / dis_wp_anterior;
        }

        // Setpoint de velocidad
        pos_tar.velocity.x = vec_x;
        pos_tar.velocity.y = vec_y;
        pos_tar.velocity.z = vec_z;

        // Determinando el yaw (se mide desde el eje x, alrededor de z es positivo)
        if (current_wp > 0 && last_wp_reached == 0){ // Si no estamos en el primer  waypoint y no hemos alcanzado el ultimo
            pos_tar.yaw = atan2(pos_tar.position.y - y, pos_tar.position.x - x);
        }
        else if (current_wp == 0 || last_wp_reached == 1){ // Si estamos en el primer wp o ya alcanzamos el ultimo
            pos_tar.yaw = 0.0;
        }


        local_raw_pub.publish(pos_tar);
        next_wp_pub.publish(next_wp_goal);


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}