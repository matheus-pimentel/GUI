#ifndef UTILS_H
#define UTILS_H

#define PI 3.14159265

#include <utility>
#include <vector>
#include "C:\Qt\Eigen\Eigen/Dense"
#include "math.h"
#include "iostream"
#include "fstream"
#include "QMessageBox"

using namespace Eigen;
using namespace std;

typedef std::vector<std::vector<double>> matrixd;

struct matrixds
{
    matrixd matrix;
    int l;
    int c;
};

void print_matrix(matrixds matrix);
matrixds rotation_matrix(double roll, double pitch, double yaw);
matrixds transformation_matrix(double roll, double pitch, double yaw);
matrixds inv_transformation_matrix(double roll, double pitch, double yaw);
matrixds sum_matrix(matrixds a, matrixds b);
matrixds transposed_matrix(matrixds a);
matrixds product_matrix(matrixds a, matrixds b);
matrixds multiple_matrix(double a, matrixds b);
matrixds inverse_matrix(matrixds matrix);
matrixds line_matrix(matrixds matrix, int a);
matrixds column_matrix(matrixds matrix, int a);
matrixds resize_matrix(int l, int c);
matrixds mxd2mds(MatrixXd matrix);
MatrixXd mds2mxd(matrixds matrix);
matrixds read_points(string fname);
void write_points(string fname, matrixds matrix);

#endif // UTILS_H
