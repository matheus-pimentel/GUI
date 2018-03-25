#ifndef SCENEMODIFIER_H
#define SCENEMODIFIER_H

#include "utils.h"
#include "quad.h"

#include <QtCore/QObject>

#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>

#include <Qt3DExtras/QTorusMesh>
#include <Qt3DExtras/QConeMesh>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QPerVertexColorMaterial>

#include <Qt3DRender/QMesh>
#include <Qt3DRender/QObjectPicker>
#include <Qt3DRender/QPickEvent>
#include <Qt3DRender/QGeometryRenderer>
#include <Qt3DRender/QGeometry>
#include <Qt3DRender/QBuffer>
#include <Qt3DRender/QAttribute>

class scenemodifier: public QObject
{
    Q_OBJECT

private:
    Qt3DCore::QEntity *m_rootEntity;
    Qt3DCore::QEntity *m_lineEntity;
    Qt3DCore::QEntity *quad_arm1;
    Qt3DCore::QEntity *quad_arm2;
    Qt3DCore::QEntity *quad_motor1;
    Qt3DCore::QEntity *quad_motor2;
    Qt3DCore::QEntity *quad_motor3;
    Qt3DCore::QEntity *quad_motor4;
    Qt3DCore::QEntity *quad_up;

    params quad_params;

    matrixds des_state;
    matrixds old_des_state;
    matrixds old_state;
    matrixds state;

    MatrixXd a1,a2,a3;
    MatrixXd b1,b2,b3;

    bool dash_line = true;

public:
    explicit scenemodifier(Qt3DCore::QEntity *rootEntity);
    ~scenemodifier();

    /***************************
     *  Miscelaneous functions *
     ***************************/
    /**
     * @brief create_grid plot a 3D grid
     */
    void create_grid();

    /**
     * @brief create_spheres creates a sphere in a desired position
     * @param x
     * @param y
     * @param z
     */
    void create_spheres(double x, double y, double z);

    /**
     * @brief create_trajectories updates the plot of the trajectory
     */
    void create_trajectories();

    /**
     * @brief update_plot updates the plot of the quadrotor
     */
    void update_plot();

    /******************
     *  Set functions *
     ******************/
    /**
     * @brief set_params defines the parameters of the quadrotor
     * @param params_q
     */
    void set_params(params params_q);

    /**
     * @brief set_states updates the state of the quadrotor
     * @param a
     * @param b
     * @param c
     * @param d
     */
    void set_states(matrixds a,matrixds b,matrixds c,matrixds d);

public slots:
    /**
     * @brief createLines creates a new line and plot It
     * @param v0
     * @param v1
     * @param index
     * @param axis
     * @param lod_param
     */
    void createLines(const QVector3D &v0, const QVector3D &v1,
                         const unsigned int index, const bool axis, const QString &lod_param);
};

#endif // SCENEMODIFIER_H

