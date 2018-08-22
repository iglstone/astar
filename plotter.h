#ifndef PLOTTER_H
#define PLOTTER_H

#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QBasicTimer>
#include <QVector>
#include <astar.h>
#include <QString>

#include "Robot.h"

class plotter : public QWidget
{
    Q_OBJECT
public:
    explicit plotter(QWidget *parent = 0);
    ~plotter();
    int robot_step;

private:
    void paintEvent(QPaintEvent *event);
    void timerEvent(QTimerEvent *event);
    void drawAction(QPainter *painter);
    void drawMap(QPainter *painter);
    //int xyToIndex(int x, int y);
    void initRobotsStates();

    //global
    QRect rect;
    int margin;
    float xstep;
    float ystep;
    QVector <Robot *> robotsArray;

    AStar *astar;
    QPen pen;

private:
    QBasicTimer m_timer;
    int XRows;
    int YCols;
    int painter_times;

    Parameters param;

signals:

public slots:
};

#endif
