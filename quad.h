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

/**
 * @brief The params struct has the parameters of the quadrotor
 */
struct params{
    double b;
    double dt;
    double gravity;
    double Ixx;
    double Iyy;
    double Izz;
    double k;
    double l;
    double mass;
};

class quad: public QThread
{
    Q_OBJECT
private:
    controller *controlhandle;

    matrixds angular_vel;
    matrixds angular_vel_quad;
    matrixds angular_acc;
    matrixds linear_acc;
    matrixds linear_vel;
    matrixds orientation;
    matrixds position;
    matrixds R;

    matrixds b3;

    matrixds des_state;
    matrixds old_des_state;
    matrixds old_state;
    matrixds state;

    matrixds motor;
    params quad_params;
    matrixds waypoints;

    double t = 0;
    double iteration = 0;
    bool is_running = false;

protected:
    void run();

public:
    quad();
    ~quad();

    /***************************
     *  Miscelaneous functions *
     ***************************/
    /**
     * @brief init_params initializes the parameters of the quadrotor
     */
    void init_params();

    /**
     * @brief init_quad initializes the state of the quadrotor
     */
    void init_quad();

    /**
     * @brief init_waypoints initializes the waypoints
     */
    void init_waypoints();

    /**
     * @brief model updates the state of the quadrotor
     */
    void model();

    /******************
     *  Get functions *
     ******************/
    /**
     * @brief get_params returns the parameters of the quadrotor
     * @return
     */
    params get_params();

    /**
     * @brief get_waypoints returns the waypoints of the trajectory
     * @return
     */
    matrixds get_waypoints();

    /******************
     *  Set functions *
     ******************/
    /**
     * @brief set_controller defines the controller to be used
     * @param a
     */
    void set_controller(int a);

    /**
     * @brief set_params defines the parameters of the quadrotor
     * @param select
     * @param value
     */
    void set_params(int select, double value);

    /**
     * @brief set_run enables or disables the main routine
     * @param a
     */
    void set_run(bool a);

    /**
     * @brief set_waypoints defines the waypoints of a trajectory
     * @param matrix
     */
    void set_waypoints(matrixds matrix);

signals:
    /**
     * @brief emit_quadStates is a function to provides the connection between the objects of the quad and mainwindow classes
     */
    void emit_quadStates(matrixds,matrixds,matrixds,matrixds,double);
};

#endif // QUAD_H
