#include "pso.h"

pso::pso()
{
    waypoints = resize_matrix(100,5);
    waypoints.l = 1;
    controlhandle = new controller;
}

double pso::fob(double kp_thrust, double kd_thrust, double kp_moment, double kd_moment)
{
    controlhandle->set_gt_gain(kp_thrust, kd_thrust, kp_moment, kd_moment);
    controlhandle->set_controller(3);
    double erro = 0;
    double t = 0;
    matrixds position = resize_matrix(1,3);
    matrixds orientation = resize_matrix(1,3);
    matrixds linear_vel = resize_matrix(1,3);
    matrixds linear_acc = resize_matrix(1,3);
    matrixds angular_vel = resize_matrix(1,3);
    matrixds angular_vel_quad = resize_matrix(1,3);
    matrixds angular_acc = resize_matrix(1,3);
    matrixds b3 = resize_matrix(1,3);
    b3.matrix = {{0,0,1}};
    matrixds state = resize_matrix(4,3);
    matrixds des_state = resize_matrix(5,3);
    matrixds motor = resize_matrix(1,4);

    for(int i = 0; i < 500; i++){
        des_state = controlhandle->trajhandle(t);
        motor = controlhandle->update_motors(t,state);

        double roll = orientation.matrix[0][0];
        double pitch = orientation.matrix[0][1];
        double yaw = orientation.matrix[0][2];

        matrixds vectora, vectorb;
        vectora = resize_matrix(1,3);
        vectorb = resize_matrix(1,3);

        matrixds R = rotation_matrix(roll,pitch,yaw);

        double thrust = quad_params.k*(motor.matrix[0][0]+motor.matrix[0][1]+motor.matrix[0][2]+motor.matrix[0][3])/quad_params.mass;

        linear_acc = sum_matrix(multiple_matrix((-quad_params.gravity),b3),transposed_matrix(multiple_matrix(thrust,column_matrix(R,2))));
        linear_vel = sum_matrix(linear_vel,multiple_matrix(quad_params.dt,linear_acc));
        position = sum_matrix(position,multiple_matrix(quad_params.dt,linear_vel));

        double p = angular_vel_quad.matrix[0][0];
        double q = angular_vel_quad.matrix[0][1];
        double r = angular_vel_quad.matrix[0][2];

        vectora.matrix = {{((quad_params.Iyy-quad_params.Izz)*q*r/quad_params.Ixx),
                           ((quad_params.Izz-quad_params.Ixx)*p*r/quad_params.Iyy),
                           ((quad_params.Ixx-quad_params.Iyy)*p*q/quad_params.Izz)}};
        vectorb.matrix = {{(quad_params.l*quad_params.k*(motor.matrix[0][1]-motor.matrix[0][3])/quad_params.Ixx),
                           (quad_params.l*quad_params.k*(-motor.matrix[0][0]+motor.matrix[0][2])/quad_params.Iyy),
                           (quad_params.b*(motor.matrix[0][0]-motor.matrix[0][1]+motor.matrix[0][2]-motor.matrix[0][3])/quad_params.Izz)}};

        angular_acc = sum_matrix(vectora,vectorb);

        angular_vel_quad = sum_matrix(angular_vel_quad,multiple_matrix(quad_params.dt,angular_acc));
        angular_vel = transposed_matrix(product_matrix(inv_transformation_matrix(roll,pitch,yaw),transposed_matrix(angular_vel_quad)));
        orientation = sum_matrix(orientation,multiple_matrix(quad_params.dt,angular_vel));

        if(orientation.matrix[0][0] > PI){
            orientation.matrix[0][0] = orientation.matrix[0][0] - 2*PI;
        }
        if(orientation.matrix[0][0] < -PI){
            orientation.matrix[0][0] = orientation.matrix[0][0] + 2*PI;
        }
        if(orientation.matrix[0][1] > PI){
            orientation.matrix[0][1] = orientation.matrix[0][1] - 2*PI;
        }
        if(orientation.matrix[0][1] < -PI){
            orientation.matrix[0][1] = orientation.matrix[0][1] + 2*PI;
        }

        state.matrix[0] = {{position.matrix[0][0], position.matrix[0][1], position.matrix[0][2]}};
        state.matrix[1] = {{linear_vel.matrix[0][0], linear_vel.matrix[0][1], linear_vel.matrix[0][2]}};
        state.matrix[2] = {{orientation.matrix[0][0], orientation.matrix[0][1], orientation.matrix[0][2]}};
        state.matrix[3] = {{angular_vel_quad.matrix[0][0], angular_vel_quad.matrix[0][1], angular_vel_quad.matrix[0][2]}};

        t = t + quad_params.dt;
        erro = erro + pow((state.matrix[0][0]-des_state.matrix[0][0]),2) + pow((state.matrix[0][1]-des_state.matrix[0][1]),2) + pow((state.matrix[0][2]-des_state.matrix[0][2]),2) + pow((state.matrix[2][2]-des_state.matrix[3][0]),2);
        cout << "erro " << erro << endl;
    }
    return erro;
}

void pso::set_waypoints(matrixds waypoints)
{
    this->waypoints = waypoints;
    controlhandle->set_waypoints(waypoints);
}

void pso::set_params(params quad_params)
{
    this->quad_params = quad_params;
    controlhandle->set_params(quad_params.mass,quad_params.dt,quad_params.gravity,quad_params.Ixx,quad_params.Iyy,quad_params.Izz,quad_params.b,quad_params.k,quad_params.l);
}
