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
    int index;
    int forward;
    QVector <posXY> path; //will not use first
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

private:
    void paintEvent(QPaintEvent *event);
    void timerEvent(QTimerEvent *event);
    void drawGrid(QPainter *);
    int xyToIndex(int x, int y);
    void testAStar();

    int XRows;
    int YCols;

    int forward;

    robot rb ;

    //global
    QRect rect;
    int margin;
    float xstep;
    float ystep;
    QVector <posXY> posArray;

    AStar *astar;

private:
    QBasicTimer m_timer;
    int m_nStep;

signals:

public slots:
};

#endif
