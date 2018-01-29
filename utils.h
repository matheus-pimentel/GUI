#ifndef UTILS_H
#define UTILS_H

#include <utility>
#include <vector>

typedef std::vector<std::vector<double>> matrixd;

struct matrixds
{
    matrixd matrix;
    int l;
    int c;
};

struct params{
    double mass;
    double l;
    double b;
    double k;
    double gravity;
    double dt;
    double Ixx;
    double Iyy;
    double Izz;
    double I[3][3];
};

void print_matrix(matrixds matrix);
matrixds rotation_matrix(double roll, double pitch, double yaw);
matrixds tranformation_matrix(double roll, double pitch, double yaw);
matrixds inv_transformation_matrix(double roll, double pitch, double yaw);
matrixds sum_matrix(matrixds a, matrixds b);
matrixds transposed_matrix(matrixds a);
matrixds product_matrix(matrixds a, matrixds b);

#endif // UTILS_H
