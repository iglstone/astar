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
/*
enum ROBOT_ForWord{
    ForWord_Self    = 0,
    ForWord_Front   = 1,
    ForWord_Up      = 2,
    ForWord_Back    = 3,
    ForWord_Down    = 4,
    ForWord_Unknow  = 5
};

enum ROBOT_Color{
    ROBOT_black    = 0,
    ROBOT_yellow   = 1,
    ROBOT_gray     = 2,
    ROBOT_green    = 3,
    ROBOT_red      = 4,
    ROBOT_blue     = 5
};

struct robot{
    int index_now;//now position
    int forward; //朝向, now use now
    QString name;
    int robot_id;
    int index_start;
    int index_end;
    int robot_step;
    QVector <posXY> path; //astar planner path
};
*/

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
    void drawGrid(QPainter *);
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
    int m_nStep;
    int XRows;
    int YCols;

    Robot rb;
    Parameters param;

signals:

public slots:
};

#endif
