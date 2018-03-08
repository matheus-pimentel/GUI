#ifndef PSO_H
#define PSO_H

#include "utils.h"
#include "quad.h"
#include "controller.h"

class pso
{
private:
    matrixds position;
    matrixds orientation;
    matrixds linear_vel;
    matrixds linear_acc;
    matrixds angular_vel;
    matrixds angular_vel_quad;
    matrixds angular_acc;
    matrixds motor;
    matrixds b3;
    matrixds state;
    matrixds des_state;
    controller *controlhandle;
    matrixds waypoints;
    params quad_params;
public:
    pso();
    double fob(double kp_thrust, double kd_thrust, double kp_moment, double kd_moment);
    void set_waypoints(matrixds waypoints);
    void set_params(params quad_params);
};

#endif // PSO_H
