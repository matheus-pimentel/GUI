#ifndef UTILS_H
#define UTILS_H

#define PI 3.14159265

#include <utility>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "math.h"
#include "iostream"
#include "fstream"
#include "QMessageBox"

using namespace Eigen;
using namespace std;

typedef std::vector<std::vector<double>> matrixd;

/**
 * @brief The matrixds struct has the parameters of a matrix
 */
struct matrixds
{
    int c;
    int l;
    matrixd matrix;
};

/**
 * @brief inv_transformation_matrix returns the inverse matrix of a transformation matrix
 * @param roll
 * @param pitch
 * @param yaw
 * @return
 */
matrixds inv_transformation_matrix(double roll, double pitch, double yaw);

/**
 * @brief rotation_matrix returns the rotation matrix of any Euler angles
 * @param roll
 * @param pitch
 * @param yaw
 * @return
 */
matrixds rotation_matrix(double roll, double pitch, double yaw);

/**
 * @brief transformation_matrix retunrs the transformation matrix of any Euler angles
 * @param roll
 * @param pitch
 * @param yaw
 * @return
 */
matrixds transformation_matrix(double roll, double pitch, double yaw);

/**
 * @brief inverse_matrix returns a inverse of any matrix
 * @param matrix
 * @return
 */
matrixds inverse_matrix(matrixds matrix);

/**
 * @brief multiple_matrix returns the multiplication of a matrix and a scalar number
 * @param a
 * @param b
 * @return
 */
matrixds multiple_matrix(double a, matrixds b);

/**
 * @brief product_matrix returns the product of two matrixes
 * @param a
 * @param b
 * @return
 */
matrixds product_matrix(matrixds a, matrixds b);

/**
 * @brief sum_matrix return the sum of two matrixes
 * @param a
 * @param b
 * @return
 */
matrixds sum_matrix(matrixds a, matrixds b);

/**
 * @brief transposed_matrix return the tranposed of any matrix
 * @param a
 * @return
 */
matrixds transposed_matrix(matrixds a);

/**
 * @brief column_matrix returns the desired column of a matrix
 * @param matrix
 * @param a
 * @return
 */
matrixds column_matrix(matrixds matrix, int a);

/**
 * @brief line_matrix returns the desired line of a matrix
 * @param matrix
 * @param a
 * @return
 */
matrixds line_matrix(matrixds matrix, int a);

/**
 * @brief resize_matrix resize a matrix
 * @param l
 * @param c
 * @return
 */
matrixds resize_matrix(int l, int c);


/**
 * @brief mds2mxd returns the equivalent matrix of another type matrix
 * @param matrix
 * @return
 */
MatrixXd mds2mxd(matrixds matrix);

/**
 * @brief mxd2mds returns the equivalent matrix of another type matrix
 * @param matrix
 * @return
 */
matrixds mxd2mds(MatrixXd matrix);

/**
 * @brief print_matrix print a matrix
 * @param matrix
 */
void print_matrix(matrixds matrix);

/**
 * @brief read_points return a matrix writed in a .txt file
 * @param fname
 * @return
 */
matrixds read_points(string fname);

/**
 * @brief write_points write a matrix in a .txt file
 * @param fname
 * @param matrix
 */
void write_points(string fname, matrixds matrix);

#endif // UTILS_H
