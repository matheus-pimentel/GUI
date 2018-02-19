#include "utils.h"
#include "math.h"
#include "iostream"

using namespace std;

void print_matrix(matrixds matrix)
{
    int i = 0, j = 0;
    for(i = 0; i < matrix.l; i++){
        for(j = 0; j < matrix.c; j++){
            cout << matrix.matrix[i][j] << " ";
        }
        cout << endl;
    }
    cout << endl;
}

matrixds rotation_matrix(double roll, double pitch, double yaw)
{
    matrixds R;
    R.matrix = matrixd(3, vector<double>(3, 0.0));
    R.matrix = {{(cos(yaw)*cos(pitch) - sin(roll)*sin(yaw)*sin(pitch)),(-cos(roll)*sin(yaw)),(cos(yaw)*sin(pitch) + cos(pitch)*sin(roll)*sin(yaw))},
         {(cos(pitch)*sin(yaw) + cos(yaw)*sin(roll)*sin(pitch)),(cos(roll)*cos(yaw)),(sin(yaw)*sin(pitch) - cos(pitch)*sin(roll)*cos(yaw))},
         {(-cos(roll)*sin(pitch)),(sin(roll)),(cos(roll)*cos(pitch))}};
    R.l = 3;
    R.c = 3;
    return R;
}

matrixds transformation_matrix(double roll, double pitch, double yaw)
{
    matrixds T;
    T.matrix = matrixd(3, vector<double>(3, 0.0));
    T.matrix = {{(cos(pitch)), (0), (-cos(roll)*sin(pitch))},
                {(0), (1), (sin(roll))},
                {(sin(pitch)), (0), (cos(roll)*cos(pitch))}};
    T.l = 3;
    T.c = 3;
    return T;
}

matrixds inv_transformation_matrix(double roll, double pitch, double yaw)
{
    matrixds inv;
    inv.matrix = matrixd(3,vector<double>(3,0.0));
    inv.l = 3;
    inv.c = 3;
    inv.matrix = {{(cos(pitch)), (0), (sin(pitch))},
                  {(sin(roll)*sin(pitch)/cos(roll)), (1), (-cos(pitch)*sin(roll)/cos(roll))},
                  {(-sin(pitch)/cos(roll)), (0), (cos(pitch)/cos(roll))}};
    return inv;
}

matrixds sum_matrix(matrixds a, matrixds b)
{
    int i = 0, j = 0;
    matrixds sum;
    sum.matrix = matrixd(a.l, vector<double>(a.c, 0.0));
    for(i = 0; i < a.l; i++){
        for(j = 0; j < a.c; j++){
            sum.matrix[i][j] = a.matrix[i][j] + b.matrix[i][j];
        }
    }
    sum.l = a.l;
    sum.c = a.c;
    return sum;
}

matrixds transposed_matrix(matrixds a)
{
    int i = 0, j = 0;
    matrixds transposed;
    transposed.matrix = matrixd(a.c, vector<double>(a.l, 0.0));
    for(i = 0; i < a.l; i++){
        for(j = 0; j < a.c; j++){
            transposed.matrix[j][i] = a.matrix[i][j];
        }
    }
    transposed.l = a.c;
    transposed.c = a.l;
    return transposed;
}

matrixds product_matrix(matrixds a, matrixds b)
{
    int i = 0, j = 0, k = 0;
    double sum = 0;
    matrixds product;
    product.matrix = matrixd(a.l, vector<double>(b.c, 0.0));
    if(a.c == b.l){
        for(i = 0; i < a.l; i++){
            for(j = 0; j < b.c; j++){
                for(k = 0; k < b.l; k++){
                    sum = sum + a.matrix[i][k]*b.matrix[k][j];
                }
                product.matrix[i][j] = sum;
                sum = 0;
            }
        }
    }
    product.l = a.l;
    product.c = b.c;
    return product;
}

matrixds multiple_matrix(double a, matrixds b)
{
    int i = 0, j = 0;
    matrixds multiple;
    multiple.matrix = matrixd(b.l,vector<double>(b.c,0.0));
    multiple.l = b.l;
    multiple.c = b.c;
    for(i = 0; i < b.l; i++){
        for(j = 0; j < b.c; j++){
            multiple.matrix[i][j] = a*b.matrix[i][j];
        }
    }
    return multiple;
}

matrixds inverse_matrix(matrixds matrix)
{
    int i = 0, j = 0;
    matrixds inv;
    MatrixXd matrix2(matrix.l,matrix.c);

    inv.matrix= matrixd(matrix.l, vector<double>(matrix.c, 0.0));
    inv.l = matrix.l;
    inv.c = matrix.c;

    if(matrix.l == matrix.c){
        for(i = 0; i < matrix.l; i++){
            for(j = 0; j < matrix.c; j++){
                matrix2(i,j) = matrix.matrix[i][j];
            }
        }
        FullPivLU<MatrixXd> lu(matrix2);
        MatrixXd lu2 = lu.inverse();

        for(i = 0; i < matrix.l; i++){
            for(j = 0; j < matrix.c; j++){
                inv.matrix[i][j] = lu2(i,j);
            }
        }
    }
    return inv;
}

matrixds line_matrix(matrixds matrix, int a)
{
    matrixds line;
    int i = 0;
    line.matrix = matrixd(1, vector<double>(matrix.c,0.0));
    line.l = 1;
    line.c = matrix.c;
    for(i = 0; i < matrix.c; i++){
        line.matrix[0][i] = matrix.matrix[a][i];
    }
    return line;
}

matrixds column_matrix(matrixds matrix, int a)
{
    matrixds column;
    int i = 0;
    column.matrix = matrixd(matrix.l, vector<double>(1,0.0));
    column.l = matrix.l;
    column.c = 1;
    for(i = 0; i < matrix.l; i++){
        column.matrix[i][0] = matrix.matrix[i][a];
    }
    return column;
}

matrixds resize_matrix(int l, int c)
{
    matrixds matrix;
    matrix.matrix = matrixd(l,vector<double>(c, 0.0));
    matrix.l = l;
    matrix.c = c;
    return matrix;
}

matrixds mxd2mds(MatrixXd matrix)
{
    matrixds result = resize_matrix(matrix.rows(),matrix.cols());
    int i = 0, j = 0;

    for(i = 0; i < matrix.rows(); i++){
        for(j = 0; j < matrix.cols(); j++){
            result.matrix[i][j] = matrix(i,j);
        }
    }
    return result;
}

MatrixXd mds2mxd(matrixds matrix)
{
    MatrixXd result(matrix.l,matrix.c);
    int i = 0, j = 0;

    for(i = 0; i < matrix.l; i++){
        for(j = 0; j < matrix.c; j++){
            result(i,j) = matrix.matrix[i][j];
        }
    }
    return result;
}
