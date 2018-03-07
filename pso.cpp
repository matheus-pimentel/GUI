#include "pso.h"

pso::pso()
{
    waypoints = resize_matrix(100,5);
    waypoints.l = 1;
    controlhandle = new controller;
}

double pso::fob(double kp_thrust, double kd_thrust, double kp_moment, double kd_moment)
{
    controlhandle->set_gt_gain(kp_thrust, kd_thrust, kp_moment, kd_moment);
    controlhandle->set_controller(3);
    double erro = 0;
    double t = 0;
    matrixds state = resize_matrix(4,3);
    matrixds motor = resize_matrix(1,4);

    for(int i = 0; i < 500; i++){
        motor = controlhandle->update_motors(t,state);
    }
}

void pso::set_waypoints(matrixds waypoints)
{
    this->waypoints = waypoints;
    controlhandle->set_waypoints(waypoints);
}

void pso::set_params(params quad_params)
{
    this->quad_params = quad_params;
    controlhandle->set_params(quad_params.mass,quad_params.dt,quad_params.gravity,quad_params.Ixx,quad_params.Iyy,quad_params.Izz,quad_params.b,quad_params.k,quad_params.l);
}
