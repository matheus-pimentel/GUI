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
    params quad_params;
    matrixds waypoints;

    MatrixXd gains_min;
    MatrixXd gains_max;

    double n;
    double alpha;
    double beta;
    double ksi;
    int t_max;

public:
    pso();

    /***************************
     *  Miscelaneous functions *
     ***************************/
    /**
     * @brief fob returns the errors about each receive gains
     * @param gains
     * @return
     */
    double fob(matrixds gains);

    /**
     * @brief optimize the gains through the iteration of the fobs
     */
    void optimize();

    /******************
     *  Set functions *
     ******************/
    /**
     * @brief set_control defines the controller to be used
     * @param a
     */
    void set_control(int a);

    /**
     * @brief set_params defines the parameters of the quadrotor
     * @param quad_params
     */
    void set_params(params quad_params);

    /**
     * @brief set_range_gains defines the maximum and minimum values to the gains
     */
    void set_range_gains();

    /**
     * @brief set_waypoints defines the waypoints of a trajectory
     * @param waypoints
     */
    void set_waypoints(matrixds waypoints);
};

#endif // PSO_H
