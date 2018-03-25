#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "utils.h"
#include "math.h"
#include "iostream"
#include "time.h"

using namespace std;

/**
 * @brief The linear_gain struct has the gains to the linear controller
 */
struct linear_gain
{
    double kp_xy;
    double kd_xy;
    double kp_z;
    double kd_z;
    double kp_moment;
    double kd_moment;
};

/**
 * @brief The non_linear_gain struct has the gains to the non-linear controllers
 */
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
    matrixds b3;
    matrixds motor;
    matrixds waypoints;

    non_linear_gain gt_gain;
    linear_gain l_gain;
    non_linear_gain tu_gain;

    double b;
    double dt;
    double gravity;
    matrixds I;
    double k;
    double l;
    double mass;

    int choose_controller = 3;

    double pitch_now;
    double roll_now;

public:
    controller();

    /***************************
     *  Miscelaneous functions *
     ***************************/
    /**
     * @brief geometric_tracking calculates the velocities of the motors, for the geometric tracking controller
     * @param t
     * @param state
     */
    void geometric_tracking(double t, matrixds state);

    /**
     * @brief linear_controller calculates the velocities of the motors, for the linear controller
     * @param t
     * @param state
     */
    void linear_controller(double t, matrixds state);

    /**
     * @brief next_state return the state of the quadrotor in the next instance time
     * @param dt
     * @param state
     * @return
     */
    matrixds next_state(double dt, matrixds state);

    /**
     * @brief thrust_up_controller calculates the velocities of the motors, for the thrust up controller
     * @param t
     * @param state
     */
    void thrust_up_controller(double t, matrixds state);

    /**
     * @brief trajhandle return the desired trajectory to the quadrotor
     * @param t
     * @return
     */
    matrixds trajhandle(double t);

    /**
     * @brief update_motors update the motors with the state in a current time
     * @param t
     * @param state
     * @return
     */
    matrixds update_motors(double t, matrixds state);

    /******************
     *  Get functions *
     ******************/
    /**
     * @brief get_des_state return the desired state in a time
     * @param t
     * @return
     */
    matrixds get_des_state(double t);

    /******************
     *  Set functions *
     ******************/
    /**
     * @brief set_controller defines what controller is running
     * @param a
     */
    void set_controller(int a);

    /**
     * @brief set_gt_gain defines the gains of the geometric tracking controller
     * @param kp_thrust
     * @param kd_thrust
     * @param kp_moment
     * @param kd_moment
     */
    void set_gt_gain(double kp_thrust, double kd_thrust, double kp_moment, double kd_moment);

    /**
     * @brief set_l_gain defines the gains of the linear controller
     * @param kp_xy
     * @param kd_xy
     * @param kp_z
     * @param kd_z
     * @param kp_moment
     * @param kd_moment
     */
    void set_l_gain(double kp_xy, double kd_xy, double kp_z, double kd_z, double kp_moment, double kd_moment);

    /**
     * @brief set_params defines the parameters of the quadrotor
     * @param mass1
     * @param dt1
     * @param gravity1
     * @param Ixx
     * @param Iyy
     * @param Izz
     * @param b1
     * @param k1
     * @param l1
     */
    void set_params(double mass1, double dt1, double gravity1, double Ixx, double Iyy, double Izz, double b1, double k1, double l1);

    /**
     * @brief set_tu_gain defines the gains of the thrust up controller
     * @param kp_thrust
     * @param kd_thrust
     * @param kp_moment
     * @param kd_moment
     */
    void set_tu_gain(double kp_thrust, double kd_thrust, double kp_moment, double kd_moment);

    /**
     * @brief set_waypoints defines the waypoints of the trajectory
     * @param points
     */
    void set_waypoints(matrixds points);
};

#endif // CONTROLLER_H
