#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "utils.h"

class controller
{
private:
    matrixds waypoints;
    matrixds motor;
public:
    controller();
    matrixds trajhandle(double t);
    void set_waypoints(matrixds points);
};

#endif // CONTROLLER_H
