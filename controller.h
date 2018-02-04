#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "utils.h"

using namespace std;

class controller
{
private:
    matrixds waypoints;
    matrixds motor;
    matrixds b3;
    double kp_thrust;
    double kd_thrust;
    double kp_moment;
    double kd_moment;    

    double mass;
    double dt;
    double gravity;
    double k;
    double b;
    double l;
    matrixds I;

public:
    controller();
    matrixds trajhandle(double t);
    matrixds update_motors(double t, matrixds state);
    void geometric_tracking(double t, matrixds state);
    matrixds next_state(double dt, matrixds state);
    void set_waypoints(matrixds points);
    void set_params(double mass1, double dt1, double gravity1, double Ixx, double Iyy, double Izz, double b1, double k1, double l1);
};

#endif // CONTROLLER_H
