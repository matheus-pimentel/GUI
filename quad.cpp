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
        matrixd matriz;
        matriz = matrixd(2, vector<double>(2, 0.0));
        matriz = {{1,2},{3,4}};
        matrixd outra = {{1,2},{3,4},{5,6}};
        outra = matrixd(2, vector<double>(2, 0.0));
        outra = nothingm(matriz);
        cout << outra[0][0] << " " << outra[0][1]<< endl << outra[1][0] << " " << outra[1][1]<< endl << endl;;

        Sleep(1000);
    }
}

void quad::init_quad()
{
    position = set_points(0,0,0);
    orientation = set_points(0,0,0);
    linear_vel = set_points(0,0,0);
    linear_acc = set_points(0,0,0);
    angular_vel = set_points(0,0,0);
    angular_vel_quad = set_points(0,0,0);
    angular_acc = set_points(0,0,0);
    set_params(1,0.468);
    set_params(2,0.225);
    set_params(3,1.14*pow(10,-7));
    set_params(4,2.98*pow(10,-6));
    set_params(5,0.004856);
    set_params(6,0.004856);
    set_params(7,0.008801);
    set_params(8,9.81);
    set_params(9,0.02);
}

void quad::rotation_matrix(double roll, double pitch, double yaw)
{
    rotation_matrix_calc[0][0] = cos(yaw)*cos(pitch) - sin(roll)*sin(yaw)*sin(pitch);
    rotation_matrix_calc[0][1] = -cos(roll)*sin(yaw);
    rotation_matrix_calc[0][2] = cos(yaw)*sin(pitch) + cos(pitch)*sin(roll)*sin(yaw);
    rotation_matrix_calc[1][0] = cos(pitch)*sin(yaw) + cos(yaw)*sin(roll)*sin(pitch);
    rotation_matrix_calc[1][1] = cos(roll)*cos(yaw);
    rotation_matrix_calc[1][2] = sin(yaw)*sin(pitch) - cos(pitch)*sin(roll)*cos(yaw);
    rotation_matrix_calc[2][0] = -cos(roll)*sin(pitch);
    rotation_matrix_calc[2][1] = sin(roll);
    rotation_matrix_calc[2][2] = cos(roll)*cos(pitch);
}

void quad::tranformation_matrix(double roll, double pitch, double yaw)
{
    transformation_matrix_calc[0][0] = cos(pitch);
    transformation_matrix_calc[0][1] = 0;
    transformation_matrix_calc[0][2] = -cos(roll)*sin(pitch);
    transformation_matrix_calc[1][0] = 0;
    transformation_matrix_calc[1][1] = 1;
    transformation_matrix_calc[1][2] = sin(roll);
    transformation_matrix_calc[2][0] = sin(pitch);
    transformation_matrix_calc[2][1] = 0;
    transformation_matrix_calc[2][2] = cos(roll)*cos(pitch);
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

quad::~quad()
{

}
