#include "controller.h"

using namespace std;

controller::controller()
{
    waypoints = resize_matrix(100,5);
    waypoints.l = 1;
    b3 = resize_matrix(1,3);
    b3.matrix = {{0, 0, 1}};
    motor = resize_matrix(1,4);
    I = resize_matrix(3,3);

    matrixds l = read_points("../Quadrotor-Simulator/Config/l_gain");
    l_gain.kp_xy = l.matrix[0][0];
    l_gain.kd_xy = l.matrix[1][0];
    l_gain.kp_z = l.matrix[2][0];
    l_gain.kd_z = l.matrix[3][0];
    l_gain.kp_moment = l.matrix[4][0];
    l_gain.kd_moment = l.matrix[5][0];

    matrixds tu = read_points("../Quadrotor-Simulator/Config/tu_gain");
    tu_gain.kp_thrust = tu.matrix[0][0];
    tu_gain.kd_thrust = tu.matrix[1][0];
    tu_gain.kp_moment = tu.matrix[2][0];
    tu_gain.kd_moment = tu.matrix[3][0];

    matrixds gt = read_points("../Quadrotor-Simulator/Config/gt_gain");
    gt_gain.kp_thrust = gt.matrix[0][0];
    gt_gain.kd_thrust = gt.matrix[1][0];
    gt_gain.kp_moment = gt.matrix[2][0];
    gt_gain.kd_moment = gt.matrix[3][0];

    srand(time(NULL));
}

/***************************
 *  Miscelaneous functions *
 ***************************/

void controller::geometric_tracking(double t, matrixds state)
{
    double roll = state.matrix[2][0];
    double pitch = state.matrix[2][1];
    double yaw = state.matrix[2][2];

    MatrixXd des_state = mds2mxd(trajhandle(t));
    MatrixXd des_state1 = mds2mxd(trajhandle(t+dt));
    MatrixXd des_state2 = mds2mxd(trajhandle(t+2*dt));

    MatrixXd state1 = mds2mxd(next_state(dt,state));
    MatrixXd state2 = mds2mxd(next_state(2*dt,state));

    MatrixXd t_vector(1,3), t_vector1(1,3), t_vector2(1,3);
    t_vector = mass*(des_state.row(2) + gravity*mds2mxd(b3) + gt_gain.kp_thrust*(des_state.row(0) - mds2mxd(line_matrix(state,0))) + gt_gain.kd_thrust*(des_state.row(1) - mds2mxd(line_matrix(state,1))));
    t_vector1 = mass*(des_state1.row(2) + gravity*mds2mxd(b3) + gt_gain.kp_thrust*(des_state1.row(0) - state1.row(0)) + gt_gain.kd_thrust*(des_state1.row(1) - state1.row(1)));
    t_vector2 = mass*(des_state2.row(2) + gravity*mds2mxd(b3) + gt_gain.kp_thrust*(des_state2.row(0) - state2.row(0)) + gt_gain.kd_thrust*(des_state2.row(1) - state2.row(1)));

    MatrixXd des_ang(3,1), des_ang1(3,1), des_ang2(3,1);

    des_ang(0,0) = atan((t_vector(0,0)*sin(des_state(3,0)) - t_vector(0,1)*cos(des_state(3,0)))/(t_vector(0,2)));
    des_ang(1,0) = atan2(t_vector(0,0)*cos(des_state(3,0)) + t_vector(0,1)*sin(des_state(3,0)),t_vector(0,2)/cos(des_ang(0,0)));
    des_ang(2,0) = des_state(3,0);
    roll_now = des_ang(0,0);
    pitch_now = des_ang(1,0);

    des_ang1(0,0) = atan((t_vector1(0,0)*sin(des_state1(3,0)) - t_vector1(0,1)*cos(des_state1(3,0)))/(t_vector1(0,2)));
    des_ang1(1,0) = atan2(t_vector1(0,0)*cos(des_state1(3,0)) + t_vector1(0,1)*sin(des_state1(3,0)),t_vector1(0,2)/cos(des_ang1(0,0)));
    des_ang1(2,0) = des_state1(3,0);

    des_ang2(0,0) = atan((t_vector2(0,0)*sin(des_state2(3,0)) - t_vector2(0,1)*cos(des_state2(3,0)))/(t_vector2(0,2)));
    des_ang2(1,0) = atan2(t_vector2(0,0)*cos(des_state2(3,0)) + t_vector2(0,1)*sin(des_state2(3,0)),t_vector2(0,2)/cos(des_ang2(0,0)));
    des_ang2(2,0) = des_state2(3,0);

    MatrixXd R = mds2mxd(rotation_matrix(roll,pitch,yaw));
    MatrixXd R_des = mds2mxd(rotation_matrix(des_ang(0,0),des_ang(1,0),des_ang(2,0)));
    MatrixXd T = mds2mxd(transformation_matrix(roll,pitch,yaw));
    MatrixXd T1 = mds2mxd(transformation_matrix(state1(2,0),state1(2,1),state1(2,2)));

    MatrixXd omega = T*(des_ang1 - des_ang)/dt;
    MatrixXd omega1 = T1*(des_ang2 - des_ang1)/dt;
    MatrixXd omega_dot = (omega1 - omega)/dt;

    MatrixXd matrix_err = (R_des.transpose()*R - R.transpose()*R_des)/2;
    MatrixXd err (3,1);
    err << matrix_err(2,1), matrix_err(0,2), matrix_err(1,0);

    MatrixXd err_omega = mds2mxd(line_matrix(state,3)).transpose() - R.transpose()*R_des*omega;

    MatrixXd F = t_vector*(R*mds2mxd(transposed_matrix(b3)));

    MatrixXd omega_hat(3,3);
    omega_hat << 0, -state.matrix[3][2], state.matrix[3][1],
                state.matrix[3][2], 0, -state.matrix[3][0],
                -state.matrix[3][1], state.matrix[3][0], 0;

    MatrixXd M = -gt_gain.kp_moment*err - gt_gain.kd_moment*err_omega + omega_hat*mds2mxd(I)*mds2mxd(state).row(3).transpose() - mds2mxd(I)*(omega_hat*R.transpose()*R_des*omega - R.transpose()*R_des*omega_dot);

    MatrixXd A(4,4), B(4,1);
    A << k, k, k, k,
         0, l*k, 0, -l*k,
         -l*k, 0, l*k, 0,
         b, -b, b, -b;
    B << F(0,0), M(0,0), M(1,0), M(2,0);

    motor = transposed_matrix(mxd2mds(A.inverse()*B));
}

void controller::linear_controller(double t, matrixds state)
{
    double roll = state.matrix[2][0];
    double pitch = state.matrix[2][1];
    double yaw = state.matrix[2][2];

    MatrixXd des_state = mds2mxd(trajhandle(t));
    MatrixXd des_state1 = mds2mxd(trajhandle(t+dt));

    MatrixXd state0 = mds2mxd(state);
    MatrixXd state1 = mds2mxd(next_state(dt,state));

    double F = mass*(gravity + des_state(2,2) + l_gain.kp_z*(des_state(0,2)-state0(0,2)) + l_gain.kd_z*(des_state(1,2)-state0(1,2)));

    double r1_ddot = des_state(2,0) + l_gain.kp_xy*(des_state(0,0)-state0(0,0)) + l_gain.kd_xy*(des_state(1,0)-state0(1,0));
    double r2_ddot = des_state(2,1) + l_gain.kp_xy*(des_state(0,1)-state0(0,1)) + l_gain.kd_xy*(des_state(1,1)-state0(1,1));

    double new_r1_ddot = des_state1(2,0) + l_gain.kp_xy*(des_state1(0,0)-state1(0,0)) + l_gain.kd_xy*(des_state1(1,0)-state1(1,0));
    double new_r2_ddot = des_state1(2,1) + l_gain.kp_xy*(des_state1(0,1)-state1(0,1)) + l_gain.kd_xy*(des_state1(1,1)-state1(1,1));

    double phides = (r1_ddot*sin(state0(2,2)) - r2_ddot*cos(state0(2,2)))/gravity;
    double thetades = (r1_ddot*cos(state0(2,2)) + r2_ddot*sin(state0(2,2)))/gravity;
    roll_now = phides;
    pitch_now = thetades;

    double new_phides = (new_r1_ddot*sin(state1(2,2)) - new_r2_ddot*cos(state1(2,2)))/gravity;
    double new_thetades = (new_r1_ddot*cos(state1(2,2)) + new_r2_ddot*sin(state1(2,2)))/gravity;

    double phidot = (new_phides - phides)/dt;
    double thetadot = (new_thetades - thetades)/dt;

    MatrixXd omega_frame_inercial(3,1);
    omega_frame_inercial << phidot, thetadot, des_state(4,0);

    MatrixXd omega = mds2mxd(transformation_matrix(roll,pitch,yaw))*omega_frame_inercial;

    MatrixXd M(3,1);

    M << l_gain.kp_moment*(phides - state0(2,0)) + l_gain.kd_moment*(omega(0,0)-state0(3,0)),
         l_gain.kp_moment*(thetades - state0(2,1)) + l_gain.kd_moment*(omega(1,0)-state0(3,1)),
         l_gain.kp_moment*(des_state(3,0) - state0(2,2)) + l_gain.kd_moment*(omega(2,0)-state0(3,2));

    M = mds2mxd(I)*M;

    MatrixXd A(4,4), B(4,1);
    A << k, k, k, k,
         0, l*k, 0, -l*k,
         -l*k, 0, l*k, 0,
         b, -b, b, -b;
    B << F, M(0,0), M(1,0), M(2,0);

    motor = transposed_matrix(mxd2mds(A.inverse()*B));
}

matrixds controller::next_state(double dt, matrixds state)
{
    matrixds next, angular_vel;
    next = resize_matrix(state.l,state.c);

    double roll = state.matrix[2][0];
    double pitch= state.matrix[2][1];
    double yaw = state.matrix[2][2];

    angular_vel = transposed_matrix(product_matrix(inv_transformation_matrix(roll,pitch,yaw),transposed_matrix(line_matrix(state,3))));

    next.matrix[0] = sum_matrix(line_matrix(state,0),multiple_matrix(dt,line_matrix(state,1))).matrix[0];
    next.matrix[1] = line_matrix(state,1).matrix[0];
    next.matrix[2] = sum_matrix(line_matrix(state,2),multiple_matrix(dt,angular_vel)).matrix[0];
    next.matrix[3] = line_matrix(state,3).matrix[0];

    return next;
}

void controller::thrust_up_controller(double t, matrixds state)
{
    double roll = state.matrix[2][0];
    double pitch = state.matrix[2][1];
    double yaw = state.matrix[2][2];

    MatrixXd des_state = mds2mxd(trajhandle(t));
    MatrixXd des_state1 = mds2mxd(trajhandle(t+dt));

    MatrixXd state0 = mds2mxd(state);
    MatrixXd state1 = mds2mxd(next_state(dt,state));

    MatrixXd t_vector(1,3), t_vector1(1,3);
    t_vector = mass*(des_state.row(2) + gravity*mds2mxd(b3) + tu_gain.kp_thrust*(des_state.row(0) - state0.row(0)) + tu_gain.kd_thrust*(des_state.row(1) - state0.row(1)));
    t_vector1 = mass*(des_state1.row(2) + gravity*mds2mxd(b3) + tu_gain.kp_thrust*(des_state1.row(0) - state1.row(0)) + tu_gain.kd_thrust*(des_state1.row(1) - state1.row(1)));

    MatrixXd des_ang(3,1), des_ang1(3,1);

    des_ang(0,0) = atan((t_vector(0,0)*sin(des_state(3,0)) - t_vector(0,1)*cos(des_state(3,0)))/(t_vector(0,2)));
    des_ang(1,0) = atan2(t_vector(0,0)*cos(des_state(3,0)) + t_vector(0,1)*sin(des_state(3,0)),t_vector(0,2)/cos(des_ang(0,0)));
    des_ang(2,0) = des_state(3,0);
    roll_now = des_ang(0,0);
    pitch_now = des_ang(1,0);

    des_ang1(0,0) = atan((t_vector1(0,0)*sin(des_state1(3,0)) - t_vector1(0,1)*cos(des_state1(3,0)))/(t_vector1(0,2)));
    des_ang1(1,0) = atan2(t_vector1(0,0)*cos(des_state1(3,0)) + t_vector1(0,1)*sin(des_state1(3,0)),t_vector1(0,2)/cos(des_ang1(0,0)));
    des_ang1(2,0) = des_state1(3,0);

    MatrixXd delta_R(3,3), delta_R1(3,3);

    delta_R = mds2mxd(rotation_matrix(des_ang(0,0), des_ang(1,0), des_ang(2,0))).transpose()*mds2mxd(rotation_matrix(roll,pitch,yaw));
    delta_R1 = mds2mxd(rotation_matrix(des_ang1(0,0), des_ang1(1,0), des_ang1(2,0))).transpose()*mds2mxd(rotation_matrix(state1(2,0), state1(2,1), state1(2,2)));

    double beta = acos((delta_R(0,0) + delta_R(1,1) + delta_R(2,2) -1)/2);
    double beta1 = acos((delta_R1(0,0) + delta_R1(1,1) + delta_R1(2,2) -1)/2);

    MatrixXd K0(3,1), K1(3,1);
    MatrixXd aux(3,1), aux1(3,1);
    aux << (delta_R(2,1) - delta_R(1,2)),
           (delta_R(0,2) - delta_R(2,0)),
           (delta_R(1,0) - delta_R(0,1));
    aux1 << (delta_R1(2,1) - delta_R1(1,2)),
            (delta_R1(0,2) - delta_R1(2,0)),
            (delta_R1(1,0) - delta_R1(0,1));

    if (aux(0,0) == 0 && aux(1,0) == 0 && aux(2,0) == 0){
        K0 << 0, 0, 0;
    }
    else{
        K0 << (aux(0,0)/sqrt((aux.transpose()*aux)(0,0))),
             (aux(1,0)/sqrt((aux.transpose()*aux)(0,0))),
             (aux(2,0)/sqrt((aux.transpose()*aux)(0,0)));
    }
    if (aux1(0,0) == 0 && aux1(1,0) == 0 && aux1(2,0) == 0){
        K1 << 0, 0, 0;
    }
    else{
        K1 << (aux1(0,0)/sqrt((aux1.transpose()*aux1)(0,0))),
              (aux1(1,0)/sqrt((aux1.transpose()*aux1)(0,0))),
              (aux1(2,0)/sqrt((aux1.transpose()*aux1)(0,0)));
    }

    MatrixXd err = beta*K0;
    MatrixXd diff_err = (beta1*K1 - err)/dt;

    MatrixXd R = mds2mxd(rotation_matrix(roll,pitch,yaw));
    MatrixXd omega_hat(3,3);
    omega_hat << 0, -state.matrix[3][2], state.matrix[3][1],
                state.matrix[3][2], 0, -state.matrix[3][0],
                -state.matrix[3][1], state.matrix[3][0], 0;

    MatrixXd F = t_vector*(R*mds2mxd(transposed_matrix(b3)));

    MatrixXd M = omega_hat*mds2mxd(I)*mds2mxd(transposed_matrix(line_matrix(state,3))) + mds2mxd(I)*(-tu_gain.kp_moment*err + tu_gain.kd_moment*diff_err);

    MatrixXd A(4,4), B(4,1);
    A << k, k, k, k,
         0, l*k, 0, -l*k,
         -l*k, 0, l*k, 0,
         b, -b, b, -b;
    B << F(0,0), M(0,0), M(1,0), M(2,0);

    motor = transposed_matrix(mxd2mds(A.inverse()*B));
}

matrixds controller::trajhandle(double t)
{
    int i = 0, j = 0;
    double t_init, t_final;

    matrixds des_state;
    des_state = resize_matrix(5,3);

    matrixds a, b, a_phi, b_phi;
    a = resize_matrix(8,8);
    b = resize_matrix(8,3);
    a_phi = resize_matrix(4,4);
    b_phi = resize_matrix(4,1);

    matrixds aux, aux_phi;
    aux = resize_matrix(3,8);
    aux_phi = resize_matrix(2,4);

    matrixds vel_waypoints = resize_matrix(waypoints.l,waypoints.c);
    for(i = 1; i < waypoints.l-1; i++){
        vel_waypoints.matrix[i] = mxd2mds(0.5*(mds2mxd(line_matrix(waypoints,i+1))-mds2mxd(line_matrix(waypoints,i-1)))).matrix[0];
    }

    double new_t;

    for(i = 0; i < waypoints.l; i++){
        if( t < waypoints.matrix[i][4]){
            t_init = 0;
            t_final = waypoints.matrix[i][4]-waypoints.matrix[i-1][4];
            new_t = t - waypoints.matrix[i-1][4];
            b.matrix = {{waypoints.matrix[i-1][0], waypoints.matrix[i-1][1], waypoints.matrix[i-1][2]},
                        {waypoints.matrix[i][0], waypoints.matrix[i][1], waypoints.matrix[i][2]},
                        {vel_waypoints.matrix[i-1][0],vel_waypoints.matrix[i-1][1],vel_waypoints.matrix[i-1][2]},
                        {vel_waypoints.matrix[i][0],vel_waypoints.matrix[i][1],vel_waypoints.matrix[i][2]},
                        {vel_waypoints.matrix[i-1][0],vel_waypoints.matrix[i-1][1],vel_waypoints.matrix[i-1][2]},
                        {vel_waypoints.matrix[i][0],vel_waypoints.matrix[i][1],vel_waypoints.matrix[i][2]},
                        {vel_waypoints.matrix[i-1][0],vel_waypoints.matrix[i-1][1],vel_waypoints.matrix[i-1][2]},
                        {vel_waypoints.matrix[i][0],vel_waypoints.matrix[i][1],vel_waypoints.matrix[i][2]},};

            b_phi.matrix = {{waypoints.matrix[i-1][3]},
                            {waypoints.matrix[i][3]},
                            {0},
                            {0}};
            break;
        }
        if( t >= waypoints.matrix[waypoints.l-1][4]){
            t_init = 0;
            t_final = t - waypoints.matrix[waypoints.l-1][4];
            new_t = t - waypoints.matrix[waypoints.l-1][4];

            b.matrix = {{waypoints.matrix[waypoints.l-1][0], waypoints.matrix[waypoints.l-1][1], waypoints.matrix[waypoints.l-1][2]},
                        {waypoints.matrix[waypoints.l-1][0], waypoints.matrix[waypoints.l-1][1], waypoints.matrix[waypoints.l-1][2]},
                        {0,0,0},
                        {0,0,0},
                        {0,0,0},
                        {0,0,0},
                        {0,0,0},
                        {0,0,0}};
            b_phi.matrix = {{waypoints.matrix[waypoints.l-1][3]},
                            {waypoints.matrix[waypoints.l-1][3]},
                            {0},
                            {0}};
        }
    }
    a.matrix = {{pow(t_init,7), pow(t_init,6), pow(t_init,5), pow(t_init,4), pow(t_init,3), pow(t_init,2), pow(t_init,1), pow(t_init,0)},
                {pow(t_final,7), pow(t_final,6), pow(t_final,5), pow(t_final,4), pow(t_final,3), pow(t_final,2), pow(t_final,1), pow(t_final,0)},
                {7*pow(t_init,6), 6*pow(t_init,5), 5*pow(t_init,4), 4*pow(t_init,3), 3*pow(t_init,2), 2*pow(t_init,1), pow(t_init,0), 0},
                {7*pow(t_final,6), 6*pow(t_final,5), 5*pow(t_final,4), 4*pow(t_final,3), 3*pow(t_final,2), 2*pow(t_final,1), pow(t_final,0), 0},
                {42*pow(t_init,5), 30*pow(t_init,4), 20*pow(t_init,3), 12*pow(t_init,2), 6*pow(t_init,1), 2*pow(t_init,0), 0, 0},
                {42*pow(t_final,5), 30*pow(t_final,4), 20*pow(t_final,3), 12*pow(t_final,2), 6*pow(t_final,1), 2*pow(t_final,0), 0, 0},
                {210*pow(t_init,4), 120*pow(t_init,3), 60*pow(t_init,2), 24*pow(t_init,1), 6*pow(t_init,0), 0, 0, 0},
                {210*pow(t_final,4), 120*pow(t_final,3), 60*pow(t_final,2), 24*pow(t_final,1), 6*pow(t_final,0), 0, 0, 0}};

    a_phi.matrix = {{pow(t_init,3), pow(t_init,2), pow(t_init,1), pow(t_init,0)},
                    {pow(t_final,3), pow(t_final,2), pow(t_final,1), pow(t_final,0)},
                    {3*pow(t_init,2), 2*pow(t_init,1), pow(t_init,0), 0},
                    {3*pow(t_final,2), 2*pow(t_final,1), pow(t_final,0), 0}};

    aux.matrix = {{pow(new_t,7), pow(new_t,6), pow(new_t,5), pow(new_t,4), pow(new_t,3), pow(new_t,2), pow(new_t,1), pow(new_t,0)},
                  {7*pow(new_t,6), 6*pow(new_t,5), 5*pow(new_t,4), 4*pow(new_t,3), 3*pow(new_t,2), 2*pow(new_t,1), pow(new_t,0), 0},
                  {42*pow(new_t,5), 30*pow(new_t,4), 20*pow(new_t,3), 12*pow(new_t,2), 6*pow(new_t,1), 2*pow(new_t,0), 0, 0}};

    aux_phi.matrix = {{pow(new_t,3), pow(new_t,2), pow(new_t,1), pow(new_t,0)},
                      {3*pow(new_t,2), 2*pow(new_t,1), pow(new_t,0), 0}};

    for(i = 0; i < 3; i++){
        for(j = 0; j < 3; j++){
            des_state.matrix[i][j] = product_matrix(line_matrix(aux,i),product_matrix(inverse_matrix(a),column_matrix(b,j))).matrix[0][0];
        }
    }
    des_state.matrix[3][0] = product_matrix(line_matrix(aux_phi,0),product_matrix(inverse_matrix(a_phi),b_phi)).matrix[0][0];
    des_state.matrix[4][0] = product_matrix(line_matrix(aux_phi,1),product_matrix(inverse_matrix(a_phi),b_phi)).matrix[0][0];

    return des_state;
}

matrixds controller::update_motors(double t, matrixds state)
{
    matrixds measured_state = resize_matrix(4,3);

    for(int i = 0; i < state.l; i++){
        for(int j = 0; j < state.c; j++){
            int signal;
            double mag;
            if((double)rand()/RAND_MAX > 0.5)
                signal = 1;
            else
                signal = -1;
            mag = 0.02;
            measured_state.matrix[i][j] = state.matrix[i][j] + signal*mag*(double)rand()/RAND_MAX;
        }
    }

    if (choose_controller == 1){
        linear_controller(t,measured_state);
    }
    else if (choose_controller == 2){
        thrust_up_controller(t,measured_state);
    }
    else{
        geometric_tracking(t,measured_state);
    }
    for(int i = 0; i < 4; i++){
        if(motor.matrix[0][i] > 2250000){
            motor.matrix[0][i] = 2250000;
        }else if (motor.matrix[0][i] < -2250000){
            motor.matrix[0][i] = -2250000;
        }
        else{}
    }
    return motor;
}

/******************
 *  Get functions *
 ******************/

matrixds controller::get_des_state(double t){
    matrixds des_state = trajhandle(t);
    des_state.matrix[3][2] = des_state.matrix[3][0];
    des_state.matrix[3][0] = roll_now;
    des_state.matrix[3][1] = pitch_now;

    return des_state;
}

/******************
 *  Set functions *
 ******************/

void controller::set_controller(int a)
{
    choose_controller = a;
}

void controller::set_gt_gain(double kp_thrust, double kd_thrust, double kp_moment, double kd_moment)
{
    gt_gain.kp_thrust = kp_thrust;
    gt_gain.kd_thrust = kd_thrust;
    gt_gain.kp_moment = kp_moment;
    gt_gain.kd_moment = kd_moment;
}

void controller::set_l_gain(double kp_xy, double kd_xy, double kp_z, double kd_z, double kp_moment, double kd_moment)
{
    l_gain.kp_xy = kp_xy;
    l_gain.kd_xy = kd_xy;
    l_gain.kp_z = kp_z;
    l_gain.kd_z = kd_z;
    l_gain.kp_moment = kp_moment;
    l_gain.kd_moment = kd_moment;
}

void controller::set_params(double mass1, double dt1, double gravity1, double Ixx, double Iyy, double Izz, double b1, double k1, double l1)
{
    mass = mass1;
    dt = dt1;
    gravity = gravity1;
    I.matrix = {{Ixx, 0, 0},
         {0, Iyy, 0},
         {0, 0, Izz}};
    b = b1;
    k = k1;
    l = l1;
}

void controller::set_tu_gain(double kp_thrust, double kd_thrust, double kp_moment, double kd_moment)
{
    tu_gain.kp_thrust = kp_thrust;
    tu_gain.kd_thrust = kd_thrust;
    tu_gain.kp_moment = kp_moment;
    tu_gain.kd_moment = kd_moment;
}

void controller::set_waypoints(matrixds points)
{
    waypoints = points;
//    cout << mds2mxd(waypoints) << endl;
}
