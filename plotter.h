#ifndef PLOTTER_H
#define PLOTTER_H

#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QBasicTimer>
#include <QVector>
#include <astar.h>

enum ForWord{
    ForWord_Self    = 0,
    ForWord_Front   = 1,
    ForWord_Up      = 2,
    ForWord_Back    = 3,
    ForWord_Down    = 4,
    ForWord_Unknow  = 5
};

struct posXY{
    int x;
    int y;
};

struct robot{
    int index_now;//now position
    int forward; //朝向
    int index_start;
    int index_end;
    QVector <posXY> path; //astar planner path
};

class plotter : public QWidget
{
    Q_OBJECT
public:
    explicit plotter(QWidget *parent = 0);
    ~plotter();

    int getXRows();
    int getYCols();
    void setXRows(int row);
    void setYCols(int col);

    posXY indexToPos(int index);
    int posToIndex(posXY pos);

    void drawCircle(int index);

    int robot_step;

private:
    void paintEvent(QPaintEvent *event);
    void timerEvent(QTimerEvent *event);
    void drawGrid(QPainter *);
    int xyToIndex(int x, int y);
    void initRobotStates();
    QVector <posXY> astarPathToMapPath(robot rob, AStar *astar);

    int XRows;
    int YCols;

    int forward;

    robot rb ;
    robot rb_run;

    //global
    QRect rect;
    int margin;
    float xstep;
    float ystep;
//    QVector <posXY> posArray;

    AStar *astar;
    QPen pen;

private:
    QBasicTimer m_timer;
    int m_nStep;

signals:

public slots:
};

#endif
