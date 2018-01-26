#ifndef QUAD_H
#define QUAD_H

#include"utils.h"
#include<QThread>

using namespace std;

class quad: public QThread
{
private:
    point position;
    point orientation;
    point linear_vel;
    point linear_acc;
    point angular_vel;
    point angular_vel_quad;
    point angular_acc;
    params quad_params;
    double R[3][3];
    double R_des[3][3];
    double T[3][3];
    double rotation_matrix_calc[3][3];
    double transformation_matrix_calc[3][3];
    double motor[4];
    double iteration = 0;
    point state[4];
    point des_state[5];
    bool is_running = false;
public:
    quad();
    void init_quad();
    void rotation_matrix(double roll, double pitch, double yaw);
    void tranformation_matrix(double roll, double pitch, double yaw);
    void set_params(int select, double value);
    void set_run(bool a);
    ~quad();

protected:
    void run();
};

#endif // QUAD_H
