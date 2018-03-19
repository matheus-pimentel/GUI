#ifndef PLOT_H
#define PLOT_H

#include <QVector>
#include <iostream>
using namespace std;

class QCPGraph;
class QCustomPlot;

class plot
{
public:
    plot(QCustomPlot *parent);
    void draw_graph();
    void set_x(double x){
        mx << x;
    }
    void set_y(double y){
        my << y;
    }
    void clear();
    void set_state();
    void set_des_state();
private:
    QCPGraph *m_graph;
    QVector<double> mx;
    QVector<double> my;
    QCustomPlot *m_parent;
};

#endif // PLOT_H
