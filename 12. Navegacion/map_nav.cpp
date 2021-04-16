// Recibe una PointCloud2 en el marco del dron, la convierte en PointCloud (x,y,z), obtiene el histograma primario, histograma binario e histograma binario alargado

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h> 
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <math.h>  
#include <stdio.h>

using namespace std; 
const float  PI = 3.14159265358979f;



// Guarda el waypoint goal actual
geometry_msgs::Point wp_goal;
void current_wp_cb(const geometry_msgs::Point::ConstPtr& msg){
    wp_goal = *msg;
}

// Guarda la PCL2 y la conviere a PCL
sensor_msgs::PointCloud point_cloud;
void pcl_cam_call(const sensor_msgs::PointCloud2ConstPtr& msg){

  sensor_msgs::convertPointCloud2ToPointCloud(*msg, point_cloud);
  //ROS_INFO("Points: %d",point_cloud.points.size());

}

int main(int argc, char** argv){
  ros::init(argc, argv, "map_nav");

  ros::NodeHandle node;

  tf::TransformListener listener; // TF listener para el waypoint goal
  ros::Duration(1.0).sleep();
 

  ROS_INFO("Map_nav ready...");

  //PointCloud2 in DRONE FRAME:
  ros::Subscriber pcl2_cam_sub = node.subscribe("transformed_pcl2", 1, pcl_cam_call);

  ros::Publisher pcl_pub = node.advertise<sensor_msgs::PointCloud>("pcl_base", 1);

  // Suscriptor del waypoint goal actual
  ros::Subscriber next_wp_sub = node.subscribe<geometry_msgs::Point>
          ("next_goal_wp", 20, current_wp_cb);

  ros::Rate rate(20.0);

  while (node.ok()){
    int sum = 0;
    int res = 3; //resolucion en grados realsense
    float ang_az, ang_el; // angulos de azimuth y elevación
    float fov_x = 87, fov_y= 57; //FOV en grados relasense
    int index_x, index_y;
    const int size_x = fov_x/res, size_y = fov_y/res; 
    float max_range = 10.0;

    int histo_prim[size_x][size_y]; //cuenta de puntos
    float histo_dis[size_x][size_y]; //promedio de distancias o distancia mínima

    for (int i = 0; i < size_x; i++) 
    { 
       for (int j = 0; j < size_y; j++) 
       { 
          histo_prim[i][j] = 0;
          histo_dis[i][j] = max_range; //max. rango
       } 
    }

    
    cout << "Total points: " << point_cloud.points.size() << endl;  


    // HISTOGRAMA PRIMARIO Y CAPA DE DISTANCIAS

    for (int i = 0; i < point_cloud.points.size(); i++){
      
      // Getting distance to the point
      float dis;
      dis = pow(point_cloud.points[i].x,2) + pow(point_cloud.points[i].y,2) + pow(point_cloud.points[i].z,2);
      dis = sqrtf(dis);
      //sum = sum + dis;
      if (dis >= 0.0 && dis <= max_range){
      // Getting azimuth and elevation angles
        ang_az = 180.0/PI * atan2(point_cloud.points[i].y, point_cloud.points[i].x) + fov_x/2;
        ang_el = point_cloud.points[i].z / sqrt(pow(point_cloud.points[i].x,2) + pow(point_cloud.points[i].y,2));
        ang_el = 180.0/PI * atan(ang_el) + fov_y/2;
        //ROS_INFO("Azimuth: %f,   Elevacion: %f",ang_az,ang_el);
        
        if ( ang_az >= 0.0 && ang_az <= fov_x && ang_el >= 0.0 && ang_el <= fov_y){
          sum ++;
          // Getting index for the point
          index_x = (int)ang_az/res;
          index_y = (int)ang_el/res;

          histo_prim[index_x][index_y]++; //Adding point to the historgam

          if (dis < histo_dis[index_x][index_y]){
            histo_dis[index_x][index_y] = dis; //Saving the minimum distance
          }
        }
       }
    }

    cout << "Counted points: " << sum << endl; 
    
    //Printing capa de distancias del histograma primario
    for (int j = size_y - 1; j >= 0; j--) 
    { 
       for (int i = size_x -1; i >= 0; i--) 
       { 
        if (histo_prim[i][j] == 0)
          {
            histo_dis[i][j] = 0.0;
          }
          cout << fixed << setprecision(2) << histo_dis[i][j] << " "; 
       } 
       // Newline for new row 
       cout << endl; 
    }
    cout << endl; 


    // HISTOGRAMA BINARIO

    bool histo_bin[size_x][size_y]; // Histograma binario
    float thresh = 5.0; // Limite de distancia para considerar como ocupado

    for (int i = 0; i < size_x; i++) 
    { 
       for (int j = 0; j < size_y; j++) 
       {
        if (histo_dis[i][j] != 0.0 && histo_dis[i][j] <= thresh && histo_prim[i][j] > 1) // Se considera la distancia y cuantos puntos hay en cada celda
        {
          histo_bin[i][j] = true;
        }
        else
        {
          histo_bin[i][j] = false;
        }
       }
    }
    
    // IMPRIMIR HISTOGRAMA BINARIO
    cout << endl; 
    for (int j = size_y - 1; j >= 0; j--) 
    { 
       for (int i = size_x -1; i >= 0; i--) 
       { 
          cout << histo_bin[i][j] << " "; 
       } 
       // Newline for new row 
       cout << endl; 
    }
    cout << endl; 
    cout << endl; 
    cout << endl;


    // ALARGAMIENTO HISTOGRAMA BINARIO
    bool histo_bin_a[size_x][size_y];
    float r_alarg = 0.25; // Radio de alargamiento en metros

    for (int i = 0; i < size_x; i++) 
    { 
      for (int j = 0; j < size_y; j++) 
      {
        histo_bin_a[i][j] = 0;
      }
    }


    for (int i = 0; i < size_x; i++) 
    { 
      for (int j = 0; j < size_y; j++) 
      {
        if (histo_bin[i][j] == 1)
        {
          histo_bin_a[i][j] = 1;

          float lambda = 180.0 / PI * asin(r_alarg / histo_dis[i][j]); 
          int lambda_int = (int)lambda/res; // Cuantas celdas se alarga

          for (int ii = i-lambda_int; ii <= i+lambda_int; ii++){
            for (int jj = j-lambda_int; jj <= j+lambda_int; jj++){
              if (ii >= 0 && jj >= 0 && ii < size_x && jj < size_y){
                histo_bin_a[ii][jj] = 1;
              }
            }
          }
        }
      }
    }

    // IMPRIMIR HISTOGRAMA BINARIO ALARGADO
    cout << endl; 
    for (int j = size_y - 1; j >= 0; j--) 
    { 
       for (int i = size_x -1; i >= 0; i--) 
       { 
          cout << histo_bin_a[i][j] << " "; 
       } 
       // Newline for new row 
       cout << endl; 
    }
    cout << endl; 
    cout << endl; 
    cout << endl;



    // Empieza la navegacion ////////////////////////////////////////////////////////////////////
    float yaw;

    // Mapeando pel siguiente waypoint en el histograma
    geometry_msgs::PointStamped wp_goal_stamped;
    wp_goal_stamped.header.stamp = ros::Time();
    wp_goal_stamped.header.frame_id = "map"; // El waypoint goal esta en el marco inercial
    //wp_goal_stamped.point.x = wp_goal.x;
    //wp_goal_stamped.point.y = wp_goal.y;
    //wp_goal_stamped.point.z = wp_goal.z;
    wp_goal_stamped.point.x = 5.0;
    wp_goal_stamped.point.y = 3.0;
    wp_goal_stamped.point.z = 3.0;

    // Transformando waypoint goal al marco del dron
    geometry_msgs::PointStamped wp_goal_base;

    try{
      listener.transformPoint("base_link", wp_goal_stamped, wp_goal_base); // Transformar punto al base_link
      //cout << "Punto transformado: " << wp_goal_base.point.x << ", " << wp_goal_base.point.y << ", " << wp_goal_base.point.z << endl;
      // cout << endl;
      // cout << endl;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    // Angulos de azimuth y elevacion del waypoint goal
    ang_az = 180.0/PI * atan2(wp_goal_base.point.y, wp_goal_base.point.x) + fov_x/2;
    ang_el = wp_goal_base.point.z / sqrt(pow(wp_goal_base.point.x,2) + pow(wp_goal_base.point.y,2));
    ang_el = 180.0/PI * atan(ang_el) + fov_y/2;
    ROS_INFO("Azimuth: %f,   Elevacion: %f", ang_az, ang_el);

    // Si recae en el histograma, obtener en que celda del histograma recae
    if ( ang_az >= 0.0 && ang_az <= fov_x && ang_el >= 0.0 && ang_el <= fov_y){

        index_x = (int)ang_az/res;
        index_y = (int)ang_el/res;
        ROS_INFO("Index x: %d,   Index y: %d", index_x, index_y);
    }
    else { // Si no esta dentro del histograma
        ROS_INFO("No recae en histograma, no debe hacer nada mas que girar el yaw");
        //yaw = atan2(wp_goal.y - y, wp_goal.x - x); // El yaw apunta directamente a la posicion goal
        // velocidad = 0
        // Que no haga nada de lo siguiente, que aqui acabe la iteracion
    }


    // Revisa si el histograma binario alargado esta vacio
    int histo_vacio = 0; // Si se mantiene en cero, no hay ningun obstaculo
    float delta_yaw; // Diferencia de yaw
    float delta_pitch; // Diferencia de pitch
    float pitch_cost; // Costo que tendra la dif. de pitch dependiendo si es up o down
    float pitch_cost_up = 5.0; // Costo del pitch cuando va a ir por arriba
    float pitch_cost_down = 10.0; // Costo del pitch cuando va a ir por abajo
    float cost_goal; // Costo de la celda hacia la posicion goal
    float delta_yaw_smooth; // Diferencia de yaw para suavizado
    float delta_pitch_smooth; // Diferencia de pitch para suavizado
    float cost_total; // Costo toal de la celda
    float goal_cost = 10.0; // Costo del componente de direccion hacia la meta (ganancia)
    float smooth_cost = 5.0; // Costo del componente de suavizado (ganancia)
    float cost_total_min = 100000; // Guardar costo de la celda con costo minimo
    int x_min; // indice en x de la celda con costo minimo
    int y_min; // indice en y de la celda con costo minimo

    for (int j = size_y - 1; j >= 0; j--) 
    { 
       for (int i = size_x -1; i >= 0; i--) 
       { 
          histo_vacio += histo_bin_a[i][j]; // SI alguna celda es 1, ya no esta vacio el histograma

          if ( histo_bin_a[i][j] == 0){ // Si la celda esta libre
            // Calcula el costo de la celda

            delta_yaw = abs(index_x - i); // Valor absoluto de la diferencia del yaw
            delta_pitch = index_y - j; // Diferencia de pitch

            if (delta_pitch <= 0.0) // Si es una dif. de pitch negativo, la celda esta arriba de la posicion goal
                pitch_cost = pitch_cost_up; 
            else  // Si es una dif. de pitch positivo, la celda esta debajo de la posicion goaL
                pitch_cost = pitch_cost_down; // Es mayor, pues no se quiere ir por abajo

            delta_pitch = abs(delta_pitch); // Valor absoluto de la dif. del pitch
            cost_goal = delta_yaw + pitch_cost * delta_pitch; // Costo a la posicion goal

            delta_yaw_smooth = abs(i - (fov_x/2)/res); // Dif. absoluta de yaw entre la posicion actual y la celda 
            delta_pitch_smooth = abs(j - (fov_y/2)/res); // Dif. absoluta de pitch entre la posicion actual y la celda 

            cost_total = goal_cost * cost_goal + smooth_cost * (delta_yaw_smooth + delta_pitch_smooth); // Costo total de la celda

            if (cost_total <= cost_total_min){ // Si su costo es el minimo, se vuelve la celda ganadora, a menos de que otra sea menor y la reemplace
              cost_total_min = cost_total;
              x_min = i;
              y_min = j;
            }

          }
       } 
    }

    if (histo_vacio == 0){ // Si el histograma binario alargado esta vacio y no hay obstaculos
        //Navegar directo al goal a velocidad maxima y ajustando el yaw
      cout << "Histograma vacio sin obstaculos" << endl;
    }
    else if (histo_vacio == size_x * size_y){ // Si el histograma binario alargado se encuentra completamente ocupado
      cout << "Histograma lleno de obstaculos" << endl;
    }
    else{ // Si hay obstaculos, velocidad menor
      cout << "Histograma parcialmente lleno de obstaculos" << endl;

    }



    pcl_pub.publish(point_cloud);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};
