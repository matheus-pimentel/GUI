#ifndef PLOT_H
#define PLOT_H

#include <QVector>
#include <iostream>
using namespace std;

class QCPGraph;
class QCustomPlot;

class plot
{
private:
    QCPGraph *m_graph;
    QCustomPlot *m_parent;
    QVector<double> mx;
    QVector<double> my;

public:
    plot(QCustomPlot *parent);

    /***************************
     *  Miscelaneous functions *
     ***************************/
    /**
     * @brief clear mx and my variables
     */
    void clear();

    /**
     * @brief draw_graph add a new data to the m_graph
     */
    void draw_graph();

    /******************
     *  Set functions *
     ******************/
    /**
     * @brief set_des_state defines the desired state in a current time
     */
    void set_des_state();

    /**
     * @brief set_state defines the state in a current time
     */
    void set_state();

    /**
     * @brief set_x add a new value to the mx vector
     * @param x
     */
    void set_x(double x);

    /**
     * @brief set_y add a new value to the my vector
     * @param y
     */
    void set_y(double y);
};

#endif // PLOT_H
