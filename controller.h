#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "utils.h"

using namespace std;

struct linear_gain
{
    double kp_xy;
    double kd_xy;
    double kp_z;
    double kd_z;
    double kp_moment;
    double kd_moment;
};


struct non_linear_gain
{
    double kp_thrust;
    double kd_thrust;
    double kp_moment;
    double kd_moment;
};

class controller
{
private:
    matrixds waypoints;
    matrixds motor;
    matrixds b3;
    int choose_controller = 3;
    non_linear_gain gt_gain;
    non_linear_gain tu_gain;
    linear_gain l_gain;
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
    void set_controller(int a);
    void linear_controller(double t, matrixds state);
    void thrust_up_controller(double t, matrixds state);
    void geometric_tracking(double t, matrixds state);
    matrixds next_state(double dt, matrixds state);
    void set_waypoints(matrixds points);
    void set_params(double mass1, double dt1, double gravity1, double Ixx, double Iyy, double Izz, double b1, double k1, double l1);
    void set_gt_gain(double kp_thrust, double kd_thrust, double kp_moment, double kd_moment);
};

#endif // CONTROLLER_H
