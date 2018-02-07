#ifndef QUAD_H
#define QUAD_H

#include "utils.h"
#include "controller.h"
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
    void emit_quadStates(matrixds);

public:
    quad();
    void init_quad();
    void model();
    void set_params(int select, double value);
    void set_waypoints(matrixds matrix);
    matrixds get_waypoints();
    void set_run(bool a);
    ~quad();

protected:
    void run();
};

#endif // QUAD_H
