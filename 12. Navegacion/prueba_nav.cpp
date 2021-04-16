#include <stdio.h>
#include <stdlib.h>   
#include <iostream>
#include <math.h>  
#include <stdio.h>
#include <iostream>
#include <iomanip> 

using namespace std;

#define GREEN   "\033[32m"      /* Green */
#define RED     "\033[31m"      /* Red */
#define YELLOW  "\033[33m"      /* Yellow */
#define RESET   "\033[0m"

int main()
{
        string color = RESET;
        int index_x = 0; // Goal x
        int index_y = 9; // Goal y

        int fov_x = 87;
        int fov_y = 57;
        int res = 3;
        const int size_x = fov_x/res, size_y = fov_y/res; 
        // histo_bin [fov_x/res][fov_y/res]
        bool histo_bin_a[size_x][size_y];


        for (int j = size_y - 1; j >= 0; j--) 
        { 
           for (int i = size_x -1; i >= 0; i--) 
           {
            if (j > 6 && j < 18)
              histo_bin_a[i][j] = 1;
            else 
              histo_bin_a[i][j] = 0;

            if (i == index_x && j == index_y)
              color = GREEN;
            else if (histo_bin_a[i][j] == 1)
               color = RED;
            else
                color = RESET;
            cout << color << histo_bin_a[i][j] << " " << RESET; 
           }
           cout << endl; 
        }
        cout << endl; 
        cout << endl; 




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
        float smooth_cost = 9.0; // Costo del componente de suavizado (ganancia)
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

                // if (cost_total < 10.0 )
                //   cout << fixed << setprecision(2) << "   " << cost_total << " ";
                // else if (cost_total < 100.0 )
                //   cout << fixed << setprecision(2) << "  " << cost_total << " ";
                // else if (cost_total < 1000.0 )
                //   cout << fixed << setprecision(2) << " " << cost_total << " ";
                // else 
                //   cout << fixed << setprecision(2) << cost_total << " ";

                if (cost_total <= cost_total_min){ // Si su costo es el minimo, se vuelve la celda ganadora, a menos de que otra sea menor y la reemplace
                  cost_total_min = cost_total;
                  x_min = i;
                  y_min = j;
                }

              }

           }
           // cout << endl;  
        }
        // cout << endl;
        // cout << endl;


        for (int j = size_y - 1; j >= 0; j--) 
        { 
           for (int i = size_x -1; i >= 0; i--) 
           { 
            if ( histo_bin_a[i][j] == 0){
              color = RESET;
              if (i == index_x && j == index_y)
                color = GREEN;
              else if (i == x_min && j == y_min)
                color = YELLOW;

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

              if (cost_total < 10.0 )
                cout << color << fixed << setprecision(2) << "   " << cost_total << " " << RESET;
              else if (cost_total < 100.0 )
                cout << color << fixed << setprecision(2) << "  " << cost_total << " " << RESET;
              else if (cost_total < 1000.0 )
                cout << color << fixed << setprecision(2) << " " << cost_total << " " << RESET;
              else 
                cout << color << fixed << setprecision(2) << cost_total << " " << RESET;

            }
            else{ 
              color = RED;
              if (i == index_x && j == index_y)
                color = GREEN;
              cout << color << "------- " << RESET;
            }
           }
           cout << endl;  
        }
        cout << endl;
        cout << endl;

        cout << "Celda del waypoint goal: ["<< index_x << ", " << index_y << "]" << endl;
        cout << "Celda ganadora: [" << x_min << ", " << y_min << "]" << endl;

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


    return 0;
}