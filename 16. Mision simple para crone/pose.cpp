
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
#include <tf/transform_datatypes.h>
#include <math.h>  
#include <stdio.h>

using namespace std; 
const float  PI = 3.14159265358979f;

// Guarda el estado actual del dron: conectado, armado, modo de vuelo (manual, offboard, land, etc.)
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// Guarda la posicion local del dron en ENU
geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

// Guarda el waypoint goal actual
geometry_msgs::Point wp_goal;
void current_wp_cb(const geometry_msgs::Point::ConstPtr& msg){
    wp_goal = *msg;
}

// Guarda la secuencia del waypoint goal actual
geometry_msgs::Point wp_seq;
void seq_wp_cb(const geometry_msgs::Point::ConstPtr& msg){
    wp_seq = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_handler");
    ros::NodeHandle nh;

    // Suscriptor del estado del dron
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    // Publica en set_position/local (posicion, cuaternion)
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);;

    // Suscriptor de la pose local
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);

    // Suscriptor del waypoint goal actual
    ros::Subscriber next_wp_sub = nh.subscribe<geometry_msgs::Point>
            ("next_goal_wp", 20, current_wp_cb);
    // Suscriptor de la secuencia del waypoint goal actual
    ros::Subscriber seq_wp_sub = nh.subscribe<geometry_msgs::Point>
            ("seq_wp", 20, seq_wp_cb);
   


    // Rate de 20 Hz
    ros::Rate rate(20.0);
    // Espera por conexion con la Pixhawk
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    int current_wp = 0; // El waypoint goal actual 
    float dis_wp = 0.0; // Distancia al waypoint goal actual
    float x, y, z; // Guardar la pose actual
    float qx, qy, qz, qw; // Guardar la orientacion actual en quaternion
    double roll, pitch, yaw; // Guardar la orientacion actual en Euler
    double yaw_goal; // Yaw hacia el sigueinte waypoint goal
    float radius_acc = 0.5; // Radio de aceptación para llegar a un waypoint 
    float vel_max = 3.0; // Magnitud de velocidad maxima
    float radius_freno = 2.5; // A que distancia del waypoint va a empezar a frenar en metros
    int last_wp_reached = 0; // para saber si ya se llego al ultimo waypoint de la mision
    float vec_x = 0.0; // Componente X de la velocidad 
    float vec_y = 0.0; // Componente Y de la velocidad
    float vec_z = 0.0; // Componente Z de la velocidad


    // Inicializa el waypoint Goal
    wp_goal.x = 0.0;
    wp_goal.y = 0.0;
    wp_goal.z = 0.0;
    // Inicializa la secuencial de waypoint actual
    wp_seq.x = 0.0;
    wp_seq.y = 0.0;
    wp_seq.z = 0.0;

    // Inicializa el setpoint de posicion y orientacion que le manda a la Pixhawk
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    // Envia 100 puntos antes de iniciar
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose); // Publica pose inicial
        ros::spinOnce();
        rate.sleep();
    }

    // Mientras el dron no esté armado
    while(ros::ok() && !current_state.armed){
        local_pos_pub.publish(pose); // Publica pose inicial
        ros::spinOnce();
        rate.sleep();
    }


    while(ros::ok() && wp_seq.y > wp_seq.x){ // Mientras no se haya alcanzado el ultimo waypoint


        // Posicion actual del dron
        x = current_pose.pose.position.x; // Posicion actual del dron en X
        y = current_pose.pose.position.y; // Posicion actual del dron en Y
        z = current_pose.pose.position.z; // Posicion actual del dron en Z

        // Setpoint de posicion hacia el siguiente waypoint goal 
        pose.pose.position.x = wp_goal.x;
        pose.pose.position.y = wp_goal.y;
        pose.pose.position.z = wp_goal.z;
        //pose.pose.position.z = wp_goal.z -  (current_alt - z);

        //Orientacion actual del dron
        qx = current_pose.pose.orientation.x;
        qy = current_pose.pose.orientation.y;
        qz = current_pose.pose.orientation.z;
        qw = current_pose.pose.orientation.w;
        tf::Quaternion q(qx, qy, qz, qw);
        tf::Matrix3x3 m(q);

        m.getRPY(roll, pitch, yaw); // Convierte de quaternion a Euler
        ROS_INFO("Yaw: %f", yaw);


        // Determinando el yaw (se mide desde el eje x, alrededor de z es positivo) en ENU
        if (wp_seq.x > 0 && wp_seq.y > wp_seq.x){ // Si no esta en el primer  waypoint y no ha alcanzado el ultimo (cuando alcanza al ultimo wp_seq.x == wp_seq.y)
            yaw_goal = atan2(wp_goal.y - y, wp_goal.x - x);
        }
        else { // Si estamos en el primer wp o ya alcanzamos el ultimo
            yaw_goal = yaw; // Mantener yaw
        }

        tf::Quaternion q_goal;
        q_goal.setRPY(0.0, 0.0, yaw_goal); // Convierte el yaw deseado a quaternion

        // Setpoint de orientacion 
        pose.pose.orientation.x = q_goal.x();
        pose.pose.orientation.y = q_goal.y();
        pose.pose.orientation.z = q_goal.z();
        pose.pose.orientation.w = q_goal.w();

        local_pos_pub.publish(pose); // Publica la posicion y orientacion deseadas

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
} 
