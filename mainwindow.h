#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "quad.h"

namespace Ui {
class mainwindow;
}

class mainwindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit mainwindow(QWidget *parent = 0);
    ~mainwindow();

private slots:
    void on_start_quad_clicked();
    void on_change_params_clicked();

private:
    quad quadrotor;
    bool quad_isrunning = false;
    Ui::mainwindow *ui;

};

#endif // MAINWINDOW_H
