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

#endif // PLOTTER_H


//http://bbs.csdn.net/topics/390538754

//// class plotrer 基本没做改动，只是改了MainWindow让我能编译
//// plotter.h
//class plotter : public QWidget
//{
//    Q_OBJECT                    // 你的源代码里居然没有这行，这到还行
//public:                            // 没有这行你怎么编译的？？？
//    plotter(QWidget *parent=0);
//private:
//    void paintEvent(QPaintEvent *event);
//    void drawGrid(QPainter *);
//};

//// plotter.cpp
//plotter::plotter(QWidget *parent):QWidget(parent)
//{
//}

//void plotter::paintEvent(QPaintEvent *event)
//{
//    QPainter painter(this);
//    drawGrid(&painter);
//    painter.setPen(Qt::black);
//    painter.drawLine(0,0,100,100);

//}

//void plotter::drawGrid(QPainter *painter)
//{

//    painter->drawLine(0,0,100,100);
//    int Margin=40;//边缘
//        QRect rect;
//        //取得绘图区域，大小要减去旁白
//        //rect=QRect(Margin+300,Margin+300,width()-Margin-700,height()-Margin-500);
//        rect=QRect(Margin+25,Margin,width()-2*Margin-10,height()-2*Margin);
//        for(int i=0;i<=20;i++)
//        {
//        int x=rect.left()+(i*(rect.width()-1)/20);
//        painter->drawLine(x,rect.top(),x,rect.bottom());
//        }
//        for(int j=0;j<=10;j++)
//        {
//            int y=rect.bottom()-(j*(rect.height()-1)/10);
//            painter->drawLine(rect.left()-5,y,rect.right(),y);
//        }

//}

//// mainwindow.h
//class MainWindow : public QMainWindow
//{
//    Q_OBJECT
//public:
//    explicit MainWindow(QWidget *parent = 0);
//    ~MainWindow();
//private:
//    plotter *plot;
//};

//// mainwindow.cpp
//MainWindow::MainWindow(QWidget *parent) :
//    QMainWindow(parent)
//{
//    this->plot = new plotter;
//    this->setCentralWidget(this->plot);
//}

//MainWindow::~MainWindow()
//{
//}
