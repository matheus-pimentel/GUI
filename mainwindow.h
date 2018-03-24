#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVector>
#include <stdlib.h>
#include <QLocale>
#include "iostream"
#include "quad.h"
#include "utils.h"
#include "pso.h"
#include "scenemodifier.h"
#include "QMessageBox"

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

class QCustomPlot;
class plot;

class mainwindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit mainwindow(QWidget *parent = nullptr);
    void init_3dquad();
    void init_2dplot();
    void update_2dplot(matrixds state, matrixds des_state, double t);
    void clear_2dplot();
    ~mainwindow();

public slots:
    void update_quadStates(matrixds state, matrixds old_state, matrixds des_state, matrixds old_des_state, double t);

private slots:
    void on_start_quad_clicked();
    void on_reset_quad_clicked();
    void on_change_params_clicked();
    void on_change_controller_clicked();
    void on_add_waypoints_clicked();
    void on_reset_way_clicked();
    void on_optimize_gain_clicked();

private:
    quad quadrotor;
    pso optimization;
    params quad_params;
    bool quad_isrunning = false;
    Ui::mainwindow *ui;
    scenemodifier *modifier;
    Qt3DCore::QEntity *rootEntity;
    Qt3DExtras::Qt3DWindow *view;
    Qt3DRender::QCamera *cameraEntity;

    QCustomPlot *mPlot;
    plot *posx;

    QCustomPlot *m_plotx, *m_ploty, *m_plotz, *m_plotroll, *m_plotpitch, *m_plotyaw;
    plot *state_x, *state_y, *state_z, *state_roll, *state_pitch, *state_yaw;
    plot *des_state_x, *des_state_y, *des_state_z, *des_state_roll, *des_state_pitch, *des_state_yaw;
};

#endif // MAINWINDOW_H
