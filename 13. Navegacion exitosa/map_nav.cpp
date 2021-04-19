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

  //PointCloud2 in en el marco del dron:
  ros::Subscriber pcl2_cam_sub = node.subscribe("transformed_pcl2", 1, pcl_cam_call);

  // Publica una PCL en el marco del dron (no se usa para nada)
  ros::Publisher pcl_pub = node.advertise<sensor_msgs::PointCloud>("pcl_base", 1);

  // Suscriptor del waypoint goal actual
  ros::Subscriber next_wp_sub = node.subscribe<geometry_msgs::Point>("next_goal_wp", 20, current_wp_cb);

  // Publica el siguiente punto al que se desea llegar (marco inercial)
  ros::Publisher next_point_pub = node.advertise<geometry_msgs::PointStamped>("next_point_map", 20);

  // Publica la escala de la velocidad deseada, el delta yaw y si el siguiente waypoint goal se encuentra dentro del FOV de la camara (x = escala_vel, y = yaw_ref, z = wp_goal_in_FOV)
  ros::Publisher vel_yaw_pub= node.advertise<geometry_msgs::Point>("vel_yaw_ref", 20);

  ros::Rate rate(20.0);

  float fov_x = 87, fov_y= 57; //FOV en grados relasense
  int res = 3; //resolucion en grados realsense
  int win_x = 14; // Celda ganadora de la iteracion pasada
  int win_y = 9; // Celda ganadora de la iteracion  pasada

  while (node.ok()){
    int sum = 0;
    
    float ang_az, ang_el; // angulos de azimuth y elevación
    
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
    float thresh = 6.0; // Limite de distancia para considerar como ocupado

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
    bool histo_bin_a[size_x][size_y]; // Histograma binario alargado
    float r_alarg = 0.70; // Radio de alargamiento en metros

    for (int i = 0; i < size_x; i++) 
    { 
      for (int j = 0; j < size_y; j++) 
      {
        histo_bin_a[i][j] = 0; // Inicializa cada delda del histograma binario alargado
      }
    }


    for (int i = 0; i < size_x; i++) 
    { 
      for (int j = 0; j < size_y; j++) 
      {
        if (histo_bin[i][j] == 1)
        {
          histo_bin_a[i][j] = 1; // Si la celda es 1 en el hist. bin, aqui igual

          float lambda = 180.0 / PI * asin(r_alarg / histo_dis[i][j]); 
          int lambda_int = (int)lambda/res; // Cuantas celdas se alarga

          for (int ii = i-lambda_int; ii <= i+lambda_int; ii++){
            for (int jj = j-lambda_int; jj <= j+lambda_int; jj++){
              if (ii >= 0 && jj >= 0 && ii < size_x && jj < size_y){
                histo_bin_a[ii][jj] = 1; // Hace 1 las celdas a su alrededor
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



    // EMPIEZA LA NAVEGACION ////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////

    float dis_ref = 1.0; // Distancia de referencia para colocar el siguiente punto en metros
    float x_ref, y_ref, z_ref; // Coordenadas del siguiente punto en el marco del dron
    float yaw_ref; // DIferencia entre el yaw actual y el yaw del siguiente punto, sera enviado al pose_handler
    float escala_vel; // La velocidad es escalada dependiendo de la  escena, sera enviado al pose_handler
    float wp_goal_in_FOV;  // Indica si el siguiente waypoint se ubica dentro del FOV de la camara


    int histo_vacio = 0; // Revisa si el histograma binario alargado esta vacio, Si se mantiene en cero, no hay ningun obstaculo
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


    // Creando siguiente punto en el marco del dron para ser convertido al marco inercial
    geometry_msgs::PointStamped next_point_stamped;
    next_point_stamped.header.frame_id = "base_link"; // El punto esta en el marco del dron

    // Siguiente punto al marco intercial, el cual sera enviado al pose_handler
    geometry_msgs::PointStamped next_point_map;

    // Point que contiene la escala de velocidad, el delta yaw, y el wp_goal_inf_FOV
    geometry_msgs::Point vel_yaw_ref;


    // Mapeando pel siguiente waypoint goal en el histograma
    geometry_msgs::PointStamped wp_goal_stamped;
    wp_goal_stamped.header.stamp = ros::Time();
    wp_goal_stamped.header.frame_id = "map"; // El waypoint goal esta en el marco inercial (Se recibe de mission_handler)
    wp_goal_stamped.point.x = wp_goal.x;
    wp_goal_stamped.point.y = wp_goal.y;
    wp_goal_stamped.point.z = wp_goal.z;
    // wp_goal_stamped.point.x = 5.0;
    // wp_goal_stamped.point.y = 3.0;
    // wp_goal_stamped.point.z = 3.0;  // COMENTAR ESTOOOO

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
    // ROS_INFO("Azimuth: %f,   Elevacion: %f", ang_az, ang_el);

    // Si recae en el histograma, obtener en que celda del histograma recae
    if ( ang_az >= 0.0 && ang_az <= fov_x && ang_el >= 0.0 && ang_el <= fov_y){

        wp_goal_in_FOV = 1.0; // Si se encuentra dentro del FOV
        index_x = (int)ang_az/res;
        index_y = (int)ang_el/res;

        // ROS_INFO("Index x: %d,   Index y: %d", index_x, index_y);
    }
    else { // Si no esta dentro del histograma
        //ROS_INFO("No recae en histograma, no debe hacer nada mas que girar el yaw");
      
        yaw_ref = ang_az; // El delta yaw es el yaw hacia el siguiente waypoint goal
        escala_vel = 0.0; // Que no se mueva
        wp_goal_in_FOV = 0.0; // No se encuentra dentro del FOV

        // Siguiente punto para ser convertido al marco inercial
        next_point_stamped.header.stamp = ros::Time();
        next_point_stamped.point.x = 0.0; // Para que el dron no se mueva
        next_point_stamped.point.y = 0.0;
        next_point_stamped.point.z = 0.0;
    }




    if (wp_goal_in_FOV == 1.0){   // Si el siguiente waypoint goal recae en el histograma

      for (int j = size_y - 1; j >= 0; j--) 
      { 
         for (int i = size_x -1; i >= 0; i--) 
         { 

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

              delta_yaw_smooth = abs(i - win_x); // Dif. absoluta de yaw entre la celda ganadora anterior y la celda 
              delta_pitch_smooth = abs(j - win_y); // Dif. absoluta de pitch entre la celda ganadora anterior y la celda 

              cost_total = goal_cost * cost_goal + smooth_cost * (delta_yaw_smooth + delta_pitch_smooth); // Costo total de la celda

              if (cost_total <= cost_total_min){ // Si su costo es el minimo, se vuelve la celda ganadora, a menos de que otra sea menor y la reemplace
                cost_total_min = cost_total;
                x_min = i;
                y_min = j;
              }

            }
            else{
              histo_vacio ++; // SI alguna celda es 1, ya no esta vacio el histograma
            }
         } 
      }

      // Guarda las celdas ganadoras para ser comparadas en la siguiente iteracion
      win_x = x_min; 
      win_y = y_min;


      if (histo_vacio == 0){ // Si el histograma binario alargado esta vacio y no hay obstaculos
          //Navegar directo al goal a velocidad maxima y ajustando el yaw
        ang_az = 180.0/PI * atan2(wp_goal_base.point.y, wp_goal_base.point.x) + fov_x/2;
        escala_vel = 1.0;  // Se navergara a la velocidad maxima
        cout << "Histograma vacio sin obstaculos" << endl;
      }
      else if (histo_vacio == size_x * size_y){ // Si el histograma binario alargado se encuentra completamente ocupado
        escala_vel = 0.0;  // El dron no se mueve en el espacio
        cout << "Histograma lleno de obstaculos" << endl;
      }
      else{ // Si hay obstaculos, velocidad menor
        escala_vel = 0.5;  // Se navegara a una velocidad reducida
        cout << "Histograma parcialmente lleno de obstaculos" << endl;

      }

      if (histo_vacio != 0){
        // Convertir celda ganadora en angulos de azimuth y elevacion
        ang_az = res * (x_min + 0.5) - fov_x/2; // Angulo del punto central de la celda en x
        ang_el = res * (y_min + 0.5) - fov_y/2; // Angulo del punto central de la celda en y
        ang_az = ang_az * PI/180.0; // Conversion de grados a radianes
        ang_el = ang_el * PI/180.0; // Conversion de grados a radianes
      }

      // El delta del yaw actual con el yaw que apunta al siguiente punto es el angulo de Azimuth
      yaw_ref = ang_az; 

      // Obteniendo el siguiente punto en el marco del dron usando la distancia de referencia dis_ref
      z_ref = dis_ref * sin(ang_el);
      float xy_ref = dis_ref * cos(ang_el);
      y_ref = xy_ref * sin(ang_az);
      x_ref = xy_ref * cos(ang_az);

      // Siguiente punto para ser convertido al marco inercial
      next_point_stamped.header.stamp = ros::Time();
      next_point_stamped.point.x = x_ref;
      next_point_stamped.point.y = y_ref;
      next_point_stamped.point.z = z_ref;

    } // Termina el if de si el siguiente waypoint goal recae en el histograma



    // Transformando el siguiente punto al marco intercial, el cual sera enviado al pose_handler
    try{
      listener.transformPoint("map", next_point_stamped, next_point_map); // Transformar punto al map
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    // Point que contiene la escala de velocidad, el delta yaw, y el wp_goal_inf_FOV
    vel_yaw_ref.x = escala_vel;
    vel_yaw_ref.y = yaw_ref;
    vel_yaw_ref.z = wp_goal_in_FOV;


    next_point_pub.publish(next_point_map); // Publica el siguiente punto en el marco el mapa
    vel_yaw_pub.publish(vel_yaw_ref); // Publica la escala de velocidad y el delta yaw
    pcl_pub.publish(point_cloud); // Publica la PCL en el marco el dron

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};
