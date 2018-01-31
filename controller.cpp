#include "controller.h"
#include "utils.h"
#include "iostream"

using namespace std;

controller::controller()
{
    waypoints.matrix = matrixd(100, vector<double>(5,0.0));
    waypoints.l = 1;
    waypoints.c = 5;
}

matrixds controller::trajhandle(double t)
{
    int i = 0, j = 0;
    double t_init, t_final;

    matrixds des_state;
    des_state.matrix = matrixd(5, vector<double>(3, 0.0));
    des_state.l = 5; des_state.c = 3;

    matrixds a, b;
    a.matrix = matrixd(8, vector<double>(8, 0.0)); a.l = 8; a.c = 8;
    b.matrix = matrixd(8, vector<double>(3, 0.0)); b.l = 8; b.c = 3;

    matrixds aux;
    aux.matrix = matrixd(3,vector<double>(8, 0.0));
    aux.l = 3; aux.c = 8;

    for(i = 0; i < waypoints.l; i++){
        if( t < waypoints.matrix[i][4]){
            t_init = waypoints.matrix[i-1][4];
            t_final = waypoints.matrix[i][4];
            b.matrix = {{waypoints.matrix[i-1][0], waypoints.matrix[i-1][1], waypoints.matrix[i-1][2]},
                        {waypoints.matrix[i][0], waypoints.matrix[i][1], waypoints.matrix[i][2]},
                        {0,0,0},
                        {0,0,0},
                        {0,0,0},
                        {0,0,0},
                        {0,0,0},
                        {0,0,0}};
            break;
        }
        if( t >= waypoints.matrix[waypoints.l-1][4]){
            t_init = waypoints.matrix[waypoints.l-1][4];
            t_final = t;
            b.matrix = {{waypoints.matrix[waypoints.l-1][0], waypoints.matrix[waypoints.l-1][1], waypoints.matrix[waypoints.l-1][2]},
                        {waypoints.matrix[waypoints.l-1][0], waypoints.matrix[waypoints.l-1][1], waypoints.matrix[waypoints.l-1][2]},
                        {0,0,0},
                        {0,0,0},
                        {0,0,0},
                        {0,0,0},
                        {0,0,0},
                        {0,0,0}};
        }
    }
    a.matrix = {{pow(t_init,7), pow(t_init,6), pow(t_init,5), pow(t_init,4), pow(t_init,3), pow(t_init,2), pow(t_init,1), pow(t_init,0)},
                {pow(t_final,7), pow(t_final,6), pow(t_final,5), pow(t_final,4), pow(t_final,3), pow(t_final,2), pow(t_final,1), pow(t_final,0)},
                {7*pow(t_init,6), 6*pow(t_init,5), 5*pow(t_init,4), 4*pow(t_init,3), 3*pow(t_init,2), 2*pow(t_init,1), pow(t_init,0), 0},
                {7*pow(t_final,6), 6*pow(t_final,5), 5*pow(t_final,4), 4*pow(t_final,3), 3*pow(t_final,2), 2*pow(t_final,1), pow(t_final,0), 0},
                {42*pow(t_init,5), 30*pow(t_init,4), 20*pow(t_init,3), 12*pow(t_init,2), 6*pow(t_init,1), 2*pow(t_init,0), 0, 0},
                {42*pow(t_final,5), 30*pow(t_final,4), 20*pow(t_final,3), 12*pow(t_final,2), 6*pow(t_final,1), 2*pow(t_final,0), 0, 0},
                {210*pow(t_init,4), 120*pow(t_init,3), 60*pow(t_init,2), 24*pow(t_init,1), 6*pow(t_init,0), 0, 0, 0},
                {210*pow(t_final,4), 120*pow(t_final,3), 60*pow(t_final,2), 24*pow(t_final,1), 6*pow(t_final,0), 0, 0, 0}};

    aux.matrix = {{pow(t,7), pow(t,6), pow(t,5), pow(t,4), pow(t,3), pow(t,2), pow(t,1), pow(t,0)},
                  {7*pow(t,6), 6*pow(t,5), 5*pow(t,4), 4*pow(t,3), 3*pow(t,2), 2*pow(t,1), pow(t,0), 0},
                  {42*pow(t,5), 30*pow(t,4), 20*pow(t,3), 12*pow(t,2), 6*pow(t,1), 2*pow(t,0), 0, 0}};

    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++){
            des_state.matrix[i][j] = product_matrix(line_matrix(aux,i),product_matrix(inverse_matrix(a),column_matrix(b,j))).matrix[0][0];
        }
    }

    return des_state;
}

void controller::set_waypoints(matrixds points)
{
    waypoints = points;
    print_matrix(waypoints);
}

