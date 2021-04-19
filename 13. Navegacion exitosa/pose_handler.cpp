
// MPC_XY_VEL_MAX = 3.0 m/s, MPC_TILT_MX_AIR = 20 °, configurado en QGroundControl

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
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

// Guarda el siguiente punto a ser publicado en el setpoint raw
geometry_msgs::PointStamped next_point_map;
void next_point_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
    next_point_map = *msg;
}

// Guarda el siguiente punto a ser publicado en el setpoint raw
geometry_msgs::Point vel_yaw_ref;
void vel_yaw_cb(const geometry_msgs::Point::ConstPtr& msg){
    vel_yaw_ref = *msg;
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
            ("mavros/setpoint_position/local", 10);

    // Publica en setpoint_raw/local (velocidad, yaw)
    ros::Publisher local_raw_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);

    // Suscriptor de la pose local
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);

    // Suscriptor del waypoint goal actual
    ros::Subscriber next_wp_sub = nh.subscribe<geometry_msgs::Point>
            ("next_goal_wp", 20, current_wp_cb);
    // Suscriptor de la secuencia del waypoint goal actual
    ros::Subscriber seq_wp_sub = nh.subscribe<geometry_msgs::Point>
            ("seq_wp", 20, seq_wp_cb);

    // Suscriptor del siguiente punto al que se desea llegar (marco inercial)
    ros::Subscriber next_point_sub = nh.subscribe<geometry_msgs::PointStamped>
            ("next_point_map", 20, next_point_cb);
   
    // Suscriptor de la escala de la velocidad deseada, el delta yaw y si el siguiente waypoint goal se encuentra dentro del FOV de la camara (x = escala_vel, y = yaw_ref, z = wp_goal_in_FOV)
    ros::Subscriber vel_yaw_sub = nh.subscribe<geometry_msgs::Point>
            ("vel_yaw_ref", 20, vel_yaw_cb);
   

    // Rate de 10 Hz
    ros::Rate rate(10.0);

    // Espera por conexion con la Pixhawk
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    float x, y, z; // Guardar la pose actual

    // Inicializa el waypoint Goal
    wp_goal.x = 0.0;
    wp_goal.y = 0.0;
    wp_goal.z = 0.0;

    // Inicializa la secuencia de waypoint actual
    wp_seq.x = 0.0;
    wp_seq.y = 0.0;
    wp_seq.z = 0.0;


    // Inicializa el setpoint de posicion y orientacion que le manda a la Pixhawk
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;


    // Envia 100 puntos antes de iniciar (requerimiento de la Pixhawk)
    for(int i = 100; ros::ok() && i > 0; --i){
        //local_raw_pub.publish(pos_tar);
        local_pos_pub.publish(pose); // Publica pose inicial
        ros::spinOnce();
        rate.sleep();
    }

    // Mientras el dron no esté armado
    while(ros::ok() && !current_state.armed){
        //local_raw_pub.publish(pos_tar); // Sigue enviando puntos
        local_pos_pub.publish(pose); // Publica pose inicial
        ros::spinOnce();
        rate.sleep();
    }


    while(ros::ok() && wp_seq.y > wp_seq.x){ // Mientras no se haya alcanzado el ultimo waypoint


        // Setpoint de posicion hacia el siguiente waypoint goal 
        pose.pose.position.x = wp_goal.x;
        pose.pose.position.y = wp_goal.y;
        pose.pose.position.z = wp_goal.z;

        // Posicion actual del dron
        x = current_pose.pose.position.x; // Posicion actual del dron en X
        y = current_pose.pose.position.y; // Posicion actual del dron en Y
        z = current_pose.pose.position.z; // Posicion actual del dron en Z

        
        if (wp_seq.x > 0 && wp_seq.y > wp_seq.x){ // Si no esta en el primer  waypoint y no ha alcanzado el ultimo (cuando alcanza al ultimo wp_seq.x == wp_seq.y)

            // Determinando el yaw (se mide desde el eje x, alrededor de z es positivo) en ENU
            float d_yaw = atan2(wp_goal.y - y, wp_goal.x - x);
            tf::Quaternion q;
            q.setRPY(0.0, 0.0, d_yaw); // Convierte el yaw deseado a quaternion

            // Setpoint de orientacion 
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();

            if (vel_yaw_ref.x == 0.0){ // Si escala_vel (recibida desde nap_nav) es cero, el siguiente waypoint no esta en el FOV, el dron mantiene su posicion y solo gira hacia el siguiente waypoint goal
                
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = z;


            }
            else if (vel_yaw_ref.x == 1.0){ // Si escala_vel es 1, no hay obstaculos y se mueve directo hacia el siguiente waypoint goal
                // Setpoint de posicion
                pose.pose.position.x = wp_goal.x;
                pose.pose.position.y = wp_goal.y;
                pose.pose.position.z = wp_goal.z;


            }
            else{ // Si escala_vel es 0.5, hay algun obstaculo, y se mueve hacia el punto recibido de map_nav

                pose.pose.position.x = next_point_map.point.x;
                pose.pose.position.y = next_point_map.point.y;
                pose.pose.position.z = next_point_map.point.z;

            }


        }

        local_pos_pub.publish(pose); // Publica lposicion y orientacion deseadas


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
} 
