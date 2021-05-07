
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/PositionTarget.h>
#include <math.h>  
#include <stdio.h>

using namespace std; 
const float  PI = 3.14159265358979f;

// Guarda el estado actual del dron: conectado, armado, modo de vuelo (manual, offboard, land, etc.)
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// Guarda la mision enviada por la estacion terrestre
vector<mavros_msgs::Waypoint> waypoints;
void waypoints_wp(const mavros_msgs::WaypointList msg){
  if (msg.waypoints.size() > 0) // Si hay waypoints en la mision, los guarda
    waypoints = msg.waypoints;
}

// Guarda la posicion local del dron en ENU
geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_handler");
    ros::NodeHandle nh;

    // Suscriptor del estado del dron
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // Suscriptor de  los waypoints guardados en la Pixhawk
    ros::Subscriber waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 10, waypoints_wp);
    // Cliente del servicio para armar el dron
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    // Cliente del servicio para cambiar el modo de vuelo
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    // Cliente del servicio para limpiar los waypoints guardados en la Pixhawk
    ros::ServiceClient waypoint_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>
            ("mavros/mission/clear");

    // Publica el waypoint goal actual
    ros::Publisher next_wp_pub = nh.advertise<geometry_msgs::Point>
            ("next_goal_wp", 10);
    // Publica el numero de secuencia del waypoint actual y el total de waypoints
    ros::Publisher seq_wp_pub = nh.advertise<geometry_msgs::Point>
            ("seq_wp", 10);

    // Suscriptor de la pose local
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);


    // Rate de 20 Hz
    ros::Rate rate(20.0);
    // Espera por conexion con la Pixhawk
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Limpia mision de la Pixhawk
    mavros_msgs::WaypointClear wpp_clear;
    while(ros::ok() && waypoint_clear_client.call(wpp_clear) && !wpp_clear.response.success) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Mission cleared");
    
    // Para poner el dron en modo LAND
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    
    // Para armar el dron
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;


    float wp_x = 0.0;
    float wp_y = 0.0;
    float wp_z = 0.0;

    // Espera mientras no haya waypoints
    while(ros::ok() && waypoints.size() == 0){
        ros::spinOnce();
        rate.sleep();
    }

    // Cuando hay waypoints, los imprime 
    for(int i = 0; i < waypoints.size(); i++)
      {
        wp_x = waypoints[i].x_lat;
        wp_y = waypoints[i].y_long;
        wp_z = waypoints[i].z_alt;
        ROS_INFO("Waypoint %d @ %f, %f, %f", i, wp_x, wp_y, wp_z);
      }

    // Limpia de nuevo los waypoints (mision)
    while(ros::ok() && waypoint_clear_client.call(wpp_clear) && !wpp_clear.response.success) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Mission cleared again");


    int current_wp = 0; // El waypoint goal actual 
    float dis_wp = 0.0; // Distancia al waypoint goal actual
    float x, y, z; // Guardar la pose actual
    float radius_acc = 1.2; // Radio de aceptación para llegar a un waypoint 
    float vel_max = 2.5; // Magnitud de velocidad maxima
    float radius_freno = 2.5; // A que distancia del waypoint va a empezar a frenar en metros
    int last_wp_reached = 0; // para saber si ya se llego al ultimo waypoint de la mision
    float vec_x = 0.0; // Componente X de la velocidad 
    float vec_y = 0.0; // Componente Y de la velocidad
    float vec_z = 0.0; // Componente Z de la velocidad


    // Inicializa el Point Goal al primer waypoint
    geometry_msgs::Point next_wp_goal;
    next_wp_goal.x = waypoints[current_wp].x_lat;
    next_wp_goal.y = waypoints[current_wp].y_long;
    next_wp_goal.z = waypoints[current_wp].z_alt;
    
    // Incializa la secuencia de waypoints (waypoint actual y total de waypoints)
    geometry_msgs::Point seq_wp;
    seq_wp.x = current_wp; // Numero de waypoint actual
    seq_wp.y = waypoints.size(); // Total de waypoints en la mision
    seq_wp.z = 0.0; // Sin uso


    while(ros::ok()){
        if( current_state.mode == "OFFBOARD" && !current_state.armed){ // Si esta en modo "OFFBOARD" pero no armado
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){ // Arma el dron
                    ROS_INFO("Vehicle armed");
                }
        }


        x = current_pose.pose.position.x; // Posicion actual del dron en X
        y = current_pose.pose.position.y; // Posicion actual del dron en Y
        z = current_pose.pose.position.z; // Posicion actual del dron en Z

        dis_wp = pow((next_wp_goal.x - x),2) + pow((next_wp_goal.y - y),2) + pow((next_wp_goal.z -  z),2);
        dis_wp = sqrtf(dis_wp); // Calcula distancia del dron al waypoint goal actual


        if (dis_wp <= radius_acc && current_wp < waypoints.size() ){ // Si el dron esta dentro del radio de aceptacion y no ha acabado la mision
            ROS_INFO("Waypoint %d reached, out of %d", current_wp, waypoints.size() -1 ); // Imprime que ya alcanzo el waypoint

            if (current_wp == waypoints.size() - 1){ // Si ya alcanzo el ultimo waypoint de la mision

                if( current_state.mode == "OFFBOARD" && set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("lANDING..."); // Si esta en modo "OFFBOARD", lo coloca en modo "LAND"
                current_wp++; // Iguala current_wp a waypoints.size(), par aavisarle al pose_handler que ya alcanzo el ultimo waypoint
                seq_wp.x = current_wp; // Actualiza la secuencia del waypoint actual

            }
            }
            else // Si no ha llegado el ultimo waypoint de la mision
            {
        // Waypoint goal cambia al siguiente waypoint de la mision
                current_wp++;
                next_wp_goal.x = waypoints[current_wp].x_lat;
                next_wp_goal.y = waypoints[current_wp].y_long;
                next_wp_goal.z = waypoints[current_wp].z_alt;
                seq_wp.x = current_wp; // Actualiza la secuencia del waypoint actual
            }
        }


        // Si el dron ya aterrizo
        if (current_wp == waypoints.size() && !current_state.armed){
            ROS_INFO("MISSION FINISHED");
            break;
        }


        next_wp_pub.publish(next_wp_goal); // Publica el waypoint goal actual
        seq_wp_pub.publish(seq_wp); // Publica la secuencia del waypoint actual 


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
} 