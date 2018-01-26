#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "iostream"
#include "quad.h"

using namespace std;

mainwindow::mainwindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::mainwindow)
{
    ui->setupUi(this);

    ui->params_options->addItem("Mass");
    ui->params_options->addItem("L");
    ui->params_options->addItem("B");
    ui->params_options->addItem("K");
    ui->params_options->addItem("Ixx");
    ui->params_options->addItem("Iyy");
    ui->params_options->addItem("Izz");
}

mainwindow::~mainwindow()
{
    delete ui;
}
void mainwindow::on_start_quad_clicked()
{
    if(quad_isrunning == false)
    {
        ui->start_quad->setText("Stop");
        quad_isrunning = true;
        quadrotor.set_run(1);
        quadrotor.start();
    }else
    {
        ui->start_quad->setText("Play");
        quad_isrunning = false;
        quadrotor.set_run(0);
        cout << 0 << endl;
    }
}
void mainwindow::on_change_params_clicked()
{
    string quad_params_decision = ui->params_options->currentText().toUtf8().constData();
    string value_s = ui->params_value->text().toUtf8().constData();
    double value = atof(value_s.c_str());
    int num_params;

    if(quad_params_decision == "Mass")
    {
        num_params = 1;
    }else if(quad_params_decision == "L")
    {
        num_params = 2;
    }else if(quad_params_decision == "B")
    {
        num_params = 3;
    }else if(quad_params_decision == "K")
    {
        num_params = 4;
    }else if(quad_params_decision == "Ixx")
    {
        num_params = 5;
    }else if(quad_params_decision == "Iyy")
    {
        num_params = 6;
    }else if(quad_params_decision == "Izz")
    {
        num_params = 7;
    }else
    {

    }

    quadrotor.set_params(num_params,value);
}
