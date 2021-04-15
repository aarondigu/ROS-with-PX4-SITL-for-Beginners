
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

    // Inicializa el la posicion_raw
    mavros_msgs::PositionTarget pos_tar;
    pos_tar.coordinate_frame =  mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // Marco local ENU
    // Masks validas: vx,vy,vz,yaw (3015); vx,vy,z,yaw (3043);
    pos_tar.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                        mavros_msgs::PositionTarget::IGNORE_PY |
                        mavros_msgs::PositionTarget::IGNORE_PZ |
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE ; // 3015
    pos_tar.position.x = 0.0;
    pos_tar.position.y = 0.0;
    pos_tar.position.z = 0.0;
    pos_tar.velocity.x = 0.0;
    pos_tar.velocity.y = 0.0;
    pos_tar.velocity.z = 0.0;
    pos_tar.yaw = 0;


    // Envia 100 puntos antes de iniciar
    for(int i = 100; ros::ok() && i > 0; --i){
        local_raw_pub.publish(pos_tar);
        ros::spinOnce();
        rate.sleep();
    }

    // Mientras el dron no esté armado
    while(ros::ok() && !current_state.armed){
        local_raw_pub.publish(pos_tar); // Sigue enviando puntos
        ros::spinOnce();
        rate.sleep();
    }


    while(ros::ok() && wp_seq.y > wp_seq.x){ // Mientras no se haya alcanzado el ultimo waypoint
	
    	// Setpoint de posicion
    	pos_tar.position.x = wp_goal.x;
    	pos_tar.position.y = wp_goal.y;
    	pos_tar.position.z = wp_goal.z;

        // Posicion actual del dron
        x = current_pose.pose.position.x; // Posicion actual del dron en X
        y = current_pose.pose.position.y; // Posicion actual del dron en Y
        z = current_pose.pose.position.z; // Posicion actual del dron en Z

        //Orientacion actual del dron
        qx = current_pose.pose.orientation.x;
        qy = current_pose.pose.orientation.y;
        qz = current_pose.pose.orientation.z;
        qw = current_pose.pose.orientation.w;
        tf::Quaternion q(qx, qy, qz, qw);
        tf::Matrix3x3 m(q);

        m.getRPY(roll, pitch, yaw); // Convierte de quaternion a Euler
        //ROS_INFO("Yaw: %f", yaw);

        dis_wp = pow((wp_goal.x - x),2) + pow((wp_goal.y - y),2) + pow((wp_goal.z -  z),2);
        dis_wp = sqrtf(dis_wp); // Calcula distancia del dron al waypoint goal actual


        // Determinando la velocidad, si el drone esta fuera del radio de freno, usa la maxima, si está dentro, empieza a frenar 
        if (dis_wp >= radius_freno){
	    // Vector unitario apuntado al waypoint goal y de magnitud vel_max
            vec_x = vel_max * (wp_goal.x - x) / dis_wp;
            vec_y = vel_max * (wp_goal.y - y) / dis_wp;
            vec_z = vel_max * (wp_goal.z - z) / dis_wp;
        }
        else{
            // La velocidad va a disminuir conforme se acerque al punto
            vec_x = vel_max * (wp_goal.x - x) / radius_freno;
            vec_y = vel_max * (wp_goal.y - y) / radius_freno;
            vec_z = vel_max * (wp_goal.z - z) / radius_freno;
        }

        // Setpoint de velocidad
        pos_tar.velocity.x = vec_x;
        pos_tar.velocity.y = vec_y;
        pos_tar.velocity.z = vec_z;

        // Calculando magnitud de velocidad deseada
        // float vel = pow(vec_x,2) + pow(vec_y,2) + pow(vec_z,2);
        // vel = sqrt(vel);
        // ROS_INFO("Velocidad: %f", vel);

        // Determinando el yaw (se mide desde el eje x, alrededor de z es positivo) en ENU
        if (wp_seq.x > 0 && wp_seq.y > wp_seq.x){ // Si no esta en el primer  waypoint y no ha alcanzado el ultimo (cuando alcanza al ultimo wp_seq.x == wp_seq.y)
            pos_tar.yaw = atan2(wp_goal.y - y, wp_goal.x - x);
        }
        else { // Si estamos en el primer wp o ya alcanzamos el ultimo
            pos_tar.yaw = yaw; // Mantener yaw
        }


        local_raw_pub.publish(pos_tar); // Publica el target raw

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
} 
