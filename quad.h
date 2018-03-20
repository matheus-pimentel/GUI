#ifndef QUAD_H
#define QUAD_H

#include "utils.h"
#include "controller.h"
#include "iostream"
#include "math.h"
#include <QThread>
#include <QObject>

using namespace Eigen;
using namespace std;

struct params{
    double mass;
    double l;
    double b;
    double k;
    double gravity;
    double dt;
    double Ixx;
    double Iyy;
    double Izz;
};

class quad: public QThread
{
    Q_OBJECT
private:
    matrixds position;
    matrixds orientation;
    matrixds linear_vel;
    matrixds linear_acc;
    matrixds angular_vel;
    matrixds angular_vel_quad;
    matrixds angular_acc;
    params quad_params;
    matrixds R;
    matrixds motor;
    matrixds b3;
    matrixds state;
    matrixds old_state;
    matrixds des_state;
    matrixds old_des_state;
    matrixds waypoints;
    controller *controlhandle;
    double t = 0;
    double iteration = 0;
    bool is_running = false;
signals:
    void emit_quadStates(matrixds,matrixds,matrixds,matrixds,double);
public:
    quad();
    void init_quad();
    void init_waypoints();
    void init_params();
    void model();
    void set_params(int select, double value);
    params get_params();
    void set_waypoints(matrixds matrix);
    void set_controller(int a);
    matrixds get_waypoints();
    void set_run(bool a);
    ~quad();

protected:
    void run();
};

#endif // QUAD_H
