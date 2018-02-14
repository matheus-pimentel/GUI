#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "quad.h"
#include "scenemodifier.h"

#include <Qt3DExtras/qt3dwindow.h>
#include <Qt3DExtras/qforwardrenderer.h>
#include <Qt3DInput/QInputAspect>
#include <Qt3DCore/qentity.h>
#include <Qt3DRender/qcamera.h>
#include <Qt3DRender/qpointlight.h>
#include <Qt3DCore/qtransform.h>
#include <Qt3DExtras/qfirstpersoncameracontroller.h>

namespace Ui {
class mainwindow;
}

class mainwindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit mainwindow(QWidget *parent = 0);
    void init_3dquad();
    ~mainwindow();

public slots:
    void update_quadStates(matrixds state, matrixds old_state, matrixds des_state, matrixds old_des_state);

private slots:
    void on_start_quad_clicked();
    void on_change_params_clicked();
    void on_add_waypoints_clicked();

private:
    quad quadrotor;
    params quad_params;
    bool quad_isrunning = false;
    Ui::mainwindow *ui;
    scenemodifier *modifier;
    Qt3DRender::QCamera *cameraEntity;
};

#endif // MAINWINDOW_H
