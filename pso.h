#ifndef PSO_H
#define PSO_H

#include "utils.h"
#include "quad.h"
#include "controller.h"
#include "time.h"

class pso
{
private:
    int control = 3;
    matrixds b3;
    controller *controlhandle;
    matrixds waypoints;
    params quad_params;
    MatrixXd gains_min;
    MatrixXd gains_max;
    double n;
    double alpha;
    double beta;
    double ksi;
    int t_max;
public:
    pso();
    double fob(matrixds gains);
    void set_waypoints(matrixds waypoints);
    void set_params(params quad_params);
    void set_control(int a);
    void set_range_gains();
    void optimize();
};

#endif // PSO_H
