#include "quad.h"
#include "windows.h"
#include "iostream"
#include "math.h"
#include "utils.h"

using namespace std;

quad::quad()
{
    init_quad();
}

void quad::run()
{
    while(is_running){
        matrixds a,b;
        a.matrix = {{1,2,3},{4,5,6}};a.l = 2;a.c = 3;
        b.matrix = {{1,2},{3,4},{5,6}};b.l = 3;b.c = 2;
        print_matrix(inv_transformation_matrix(0,0,0));
        Sleep(1000);
    }
}

void quad::init_quad()
{
    position.matrix = matrixd(1, vector<double>(3, 0.0)); position.l = 1; position.c = 3;
    orientation.matrix = matrixd(1, vector<double>(3, 0.0)); orientation.l = 1; orientation.c = 3;
    linear_vel.matrix = matrixd(1, vector<double>(3, 0.0)); linear_vel.l = 1; linear_vel.c = 3;
    linear_acc.matrix = matrixd(1, vector<double>(3, 0.0)); linear_acc.l = 1; linear_acc.c = 3;
    angular_vel.matrix = matrixd(1, vector<double>(3, 0.0)); angular_vel.l = 1; angular_vel.c = 3;
    angular_vel_quad.matrix = matrixd(1, vector<double>(3, 0.0)); angular_vel_quad.l = 1; angular_vel_quad.c = 3;
    angular_acc.matrix = matrixd(1, vector<double>(3, 0.0)); angular_acc.l = 1; angular_acc.c = 3;
    motor.matrix = matrixd(1, vector<double>(4, 0.0)); motor.l = 1; motor.c = 4;
    set_params(1,0.468);
    set_params(2,0.225);
    set_params(3,1.14*pow(10,-7));
    set_params(4,2.98*pow(10,-6));
    set_params(5,0.004856);
    set_params(6,0.004856);
    set_params(7,0.008801);
    set_params(8,9.81);
    set_params(9,0.01);
}

void quad::set_params(int select, double value)
{
    switch (select) {
    case 1:
        quad_params.mass = value;
        break;
    case 2:
        quad_params.l = value;
        break;
    case 3:
        quad_params.b = value;
        break;
    case 4:
        quad_params.k = value;
        break;
    case 5:
        quad_params.Ixx = value;
        break;
    case 6:
        quad_params.Iyy = value;
        break;
    case 7:
        quad_params.Izz = value;
        break;
    case 8:
        quad_params.gravity = value;
        break;
    case 9:
        quad_params.dt = value;
        break;
    default:
        break;
    }
}

void quad::set_run(bool a)
{
    is_running = a;
}

void quad::print_matrix(matrixds matrix)
{
    int i = 0, j = 0;
    for(i = 0; i < matrix.l; i++){
        for(j = 0; j < matrix.c; j++){
            cout << matrix.matrix[i][j] << " ";
        }
        cout << endl;
    }
    cout << endl;
}

quad::~quad()
{

}
