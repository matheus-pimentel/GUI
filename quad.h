#ifndef QUAD_H
#define QUAD_H

#include"utils.h"
#include<QThread>

using namespace std;

class quad: public QThread
{
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
    matrixds state;
    matrixds des_state;
    double t = 0;
    double dt = 0.01;
    double iteration = 0;
    bool is_running = false;
public:
    quad();
    void init_quad();
    void set_params(int select, double value);
    void set_run(bool a);
    ~quad();

protected:
    void run();
};

#endif // QUAD_H
