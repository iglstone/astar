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
    int robot_step;
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

    int robot_step;

private:
    void paintEvent(QPaintEvent *event);
    void timerEvent(QTimerEvent *event);
    void drawGrid(QPainter *);
    int xyToIndex(int x, int y);
    void initRobotsStates();
    QVector <posXY> astarPathToMapPath(robot *rob, AStar *astar);
    robot *initARobot(int start_x, int start_y, int end_x, int end_y);

    int XRows;
    int YCols;

    int forward;

    robot rb ;

    //global
    QRect rect;
    int margin;
    float xstep;
    float ystep;
    QVector <robot *> robotsArray;
//    std::vector <robot *> robotsArray;

    AStar *astar;
    QPen pen;

private:
    QBasicTimer m_timer;
    int m_nStep;

signals:

public slots:
};

#endif
