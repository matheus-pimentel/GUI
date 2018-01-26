#include "utils.h"

using namespace std;

point set_points(double vector1, double vector2, double vector3)
{
    point a;
    a.x = vector1;
    a.y = vector2;
    a.z = vector3;
    return a;
}

vectord nothing(vectord a)
{
    return a;
}

matrixd nothingm(matrixd a)
{
    return a;
}
