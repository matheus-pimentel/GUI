#include "pso.h"

pso::pso()
{
    waypoints = resize_matrix(100,5);
    waypoints.l = 1;
    controlhandle = new controller;
    n = 50;
    alpha = 0.1;
    beta = 0.001;
    ksi = 1;
    t_max = 10;
    srand(time(NULL));
}

double pso::fob(matrixds gains)
{
    controlhandle->set_controller(control);
    if(control == 1){
        controlhandle->set_l_gain(gains.matrix[0][0], gains.matrix[0][1], gains.matrix[0][2], gains.matrix[0][3], gains.matrix[0][4], gains.matrix[0][5]);
    }
    else if(control == 2){
        controlhandle->set_tu_gain(gains.matrix[0][0], gains.matrix[0][1], gains.matrix[0][2], gains.matrix[0][3]);
    }
    else if(control == 3){
        controlhandle->set_gt_gain(gains.matrix[0][0], gains.matrix[0][1], gains.matrix[0][2], gains.matrix[0][3]);
    }
    else{
        controlhandle->set_gt_gain(gains.matrix[0][0], gains.matrix[0][1], gains.matrix[0][2], gains.matrix[0][3]);
    }

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

    int vmax = (waypoints.matrix[waypoints.l-1][4]/quad_params.dt) + 10;
    for(int i = 0; i < vmax; i++){
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
    }
    return erro;
}

void pso::optimize()
{
    int t = 0;
    set_range_gains();
    MatrixXd pos;
    pos.resize(n,gains_min.cols());
    for(int i = 0; i < n; i++){
        for(int j = 0; j < gains_min.cols(); j++){
            pos(i,j) = (gains_max(0,j)-gains_min(0,j))*(double)rand()/RAND_MAX + gains_min(0,j);
        }
    }
    MatrixXd vels;
    vels = MatrixXd::Zero(n,gains_min.cols());

    MatrixXd fitness;
    fitness.resize(n,1);
    for(int i = 0; i < n; i++){
        fitness(i,0) = fob(mxd2mds(pos.row(i)));
        cout << "amostra " << i+1 << " erro " << fitness(i,0) << endl;
    }

    MatrixXd pos_star;
    pos_star.resize(n,gains_min.cols());
    pos_star = pos;
    MatrixXd fitness_star;
    fitness_star.resize(n,1);
    fitness_star = fitness;

    double best_fitness = fitness_star.minCoeff();

    MatrixXd best_pos;
    best_pos.resize(1,gains_min.cols());
    for(int i = 0; i < n; i++){
        if(fitness_star(i,0) == best_fitness){
            best_pos = pos_star.row(i);
        }
    }

    while(best_fitness > ksi && t < t_max){
        t++;
        for(int i = 0; i < n; i++){
            for(int j = 0; j < gains_min.cols(); j++){
                vels(i,j) = vels(i,j) + alpha*(best_pos(0,j) - pos(i,j))*(double)rand()/RAND_MAX + beta*(pos_star(i,j) - pos(i,j))*(double)rand()/RAND_MAX;
            }
        }
        pos = pos + vels;
        for(int i = 0; i < n; i++){
            for(int j = 0; j < gains_min.cols(); j++){
                if(pos(i,j) < gains_min(0,j)){
                    pos(i,j) = gains_min(0,j);
                }
                if(pos(i,j) > gains_max(0,j)){
                    pos(i,j) = gains_max(0,j);
                }
            }
        }
        for(int i = 0; i < n; i++){
            fitness(i,0) = fob(mxd2mds(pos.row(i)));
            cout << "amostra " << i+1 << " erro " << fitness(i,0) << endl;
        }
        for(int i = 0; i < n; i++){
            if(fitness(i,0) < fitness_star(i,0)){
                fitness_star(i,0) = fitness(i,0);
                pos_star.row(i) = pos.row(i);
            }
        }
        best_fitness = fitness_star.minCoeff();

        for(int i = 0; i < n; i++){
            if(fitness_star(i,0) == best_fitness){
                best_pos = pos_star.row(i);
            }
        }
        cout << "t " << t << endl;
        cout << "erro " << best_fitness << endl;
        cout << "pos " << best_pos.transpose() << endl;
    }
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

void pso::set_control(int a)
{
    control = a;
}

void pso::set_range_gains()
{
    if(control == 1){
        gains_min.resize(1,6);
        gains_min << 0,0,0,0,0,0;
        gains_max.resize(1,6);
        gains_max << 50,50,200,200,1000,100;
    }
    else if(control == 2){
        gains_min.resize(1,4);
        gains_min << 0,0,0,0;
        gains_max.resize(1,4);
        gains_max << 100,1,1000,1;
    }
    else if(control == 3){
        gains_min.resize(1,4);
        gains_min << 0,0,0,0;
        gains_max.resize(1,4);
        gains_max << 50,10,50,1;
    }
    else{
        gains_min.resize(1,4);
        gains_min << 0,0,0,0;
        gains_max.resize(1,4);
        gains_max << 50,10,50,1;
    }
}
