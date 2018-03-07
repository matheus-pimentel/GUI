#ifndef PSO_H
#define PSO_H

#include "utils.h"
#include "quad.h"
#include "controller.h"

class pso
{
private:
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
