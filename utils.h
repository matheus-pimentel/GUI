#ifndef UTILS_H
#define UTILS_H

#include <utility>
#include <vector>

struct point{
    double x;
    double y;
    double z;
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

typedef std::vector<double> vectord;
typedef std::vector<std::vector<double> > matrixd;

point set_points(double vector1, double vector2, double vector3);
vectord nothing(vectord);
matrixd nothingm(matrixd);

#endif // UTILS_H
