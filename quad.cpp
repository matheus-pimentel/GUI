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
        model();
        print_matrix(controlhandle.trajhandle(t));
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
    state.matrix = matrixd(4, vector<double>(3, 0.0)); state.l = 4; state.c = 3;
    old_state.matrix = matrixd(4, vector<double>(3, 0.0)); old_state.l = 4; old_state.c = 3;
    des_state.matrix = matrixd(5, vector<double>(3, 0.0)); des_state.l = 5; des_state.c = 3;
    old_des_state.matrix = matrixd(5, vector<double>(3, 0.0)); old_des_state.l = 5; old_des_state.c = 3;
    motor.matrix = matrixd(1, vector<double>(4, 0.0)); motor.l = 1; motor.c = 4;
    b3.matrix = matrixd(1,vector<double>(3,0.0)); b3.l = 1; b3.c = 3;
    b3.matrix = {{0,0,1}};
    set_params(1,0.468);
    set_params(2,0.225);
    set_params(3,1.14*pow(10,-7));
    set_params(4,2.98*pow(10,-6));
    set_params(5,0.004856);
    set_params(6,0.004856);
    set_params(7,0.008801);
    set_params(8,9.81);
    set_params(9,0.01);
    waypoints.matrix = matrixd(100,vector<double>(5,0.0)); waypoints.l = 1; waypoints.c = 5;
}

void quad::model()
{
    double roll = orientation.matrix[0][0];
    double pitch = orientation.matrix[0][1];
    double yaw = orientation.matrix[0][2];

    R = rotation_matrix(roll,pitch,yaw);
    double thrust = quad_params.k*(pow(motor.matrix[0][0],2)+pow(motor.matrix[0][1],2)+pow(motor.matrix[0][2],2)+pow(motor.matrix[0][3],2));

    linear_acc = sum_matrix(multiple_matrix((-quad_params.gravity),b3),transposed_matrix(product_matrix(R,transposed_matrix(multiple_matrix((thrust/quad_params.mass),b3)))));
    linear_vel = sum_matrix(linear_vel,multiple_matrix(quad_params.dt,linear_acc));
    position = sum_matrix(position,multiple_matrix(quad_params.dt,linear_vel));

    double p = angular_vel_quad.matrix[0][0];
    double q = angular_vel_quad.matrix[0][1];
    double r = angular_vel_quad.matrix[0][2];

    matrixds vectora, vectorb;
    vectora.l = 1; vectora.c = 3;
    vectorb.l = 1; vectorb.c = 3;
    vectora.matrix = matrixd(1,vector<double>(3,0.0));
    vectorb.matrix = matrixd(1,vector<double>(3,0.0));
    vectora.matrix = {{((quad_params.Iyy-quad_params.Izz)*q*r/quad_params.Ixx),
                       ((quad_params.Izz-quad_params.Ixx)*p*r/quad_params.Iyy),
                       ((quad_params.Ixx-quad_params.Iyy)*p*q/quad_params.Izz)}};
    vectorb.matrix = {{(quad_params.l*quad_params.k*(pow(motor.matrix[0][1],2)-pow(motor.matrix[0][3],2))/quad_params.Ixx),
                       (quad_params.l*quad_params.k*(-pow(motor.matrix[0][0],2)+pow(motor.matrix[0][2],2))/quad_params.Iyy),
                       (quad_params.b*(pow(motor.matrix[0][0],2)-pow(motor.matrix[0][1],2)+pow(motor.matrix[0][2],2)-pow(motor.matrix[0][3],2))/quad_params.Izz)}};

    angular_acc = sum_matrix(vectora,vectorb);
    angular_vel_quad = sum_matrix(angular_vel_quad,multiple_matrix(quad_params.dt,angular_acc));
    angular_vel = transposed_matrix(product_matrix(inv_transformation_matrix(roll,pitch,yaw),transposed_matrix(angular_vel_quad)));
    orientation = sum_matrix(orientation,multiple_matrix(quad_params.dt,angular_vel));

    if(orientation.matrix[0][0] > PI){
        orientation.matrix[0][0] = orientation.matrix[0][0] - 2*PI;
    }
    if(orientation.matrix[0][0] < -PI){
        orientation.matrix[0][0] = orientation.matrix[0][0] + 2*PI;
    }
    if(orientation.matrix[0][1] > PI){
        orientation.matrix[0][1] = orientation.matrix[0][1] - 2*PI;
    }
    if(orientation.matrix[0][1] < -PI){
        orientation.matrix[0][1] = orientation.matrix[0][1] + 2*PI;
    }

    old_state = state;
    state.matrix[0] = {{position.matrix[0][0], position.matrix[0][1], position.matrix[0][2]}};
    state.matrix[1] = {{linear_vel.matrix[0][0], linear_vel.matrix[0][1], linear_vel.matrix[0][2]}};
    state.matrix[2] = {{orientation.matrix[0][0], orientation.matrix[0][1], orientation.matrix[0][2]}};
    state.matrix[3] = {{angular_vel_quad.matrix[0][0], angular_vel_quad.matrix[0][1], angular_vel_quad.matrix[0][2]}};

    t = t + quad_params.dt;
    iteration++;
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

void quad::set_waypoints(matrixds matrix)
{
    waypoints.matrix[waypoints.l] = {{matrix.matrix[0][0], matrix.matrix[0][1], matrix.matrix[0][2], matrix.matrix[0][3], matrix.matrix[0][4]}};
    waypoints.l++;
    controlhandle.set_waypoints(waypoints);
}

matrixds quad::get_waypoints()
{
    return waypoints;
}

void quad::set_run(bool a)
{
    is_running = a;
}

quad::~quad()
{

}
