#include "plotter.h"
#include <iostream>
#include <QtGlobal>
#include <QDebug>
#include <QTime>
#include "Parameters.h"

plotter::plotter(QWidget *parent) : QWidget(parent)
{
    this->XRows = param.getXRows();
    this->YCols = param.getYCols();

    m_timer.start(800, this);

    margin = 35;//边缘
    robot_radius = 5;
    rect = QRect(margin, margin, this->width()-2*margin, this->height()-2*margin );//取得绘图区域，大小要减去旁白
    xstep = (float)(rect.width())/ (this->XRows -1);
    ystep = (float)(rect.height())/ (this->YCols -1);

    //path pen style
    pen.setStyle(Qt::DotLine);
    pen.setWidth(3);
    pen.setBrush(Qt::red);

    robot_step = 0;
    painter_times = 0;

//    this->astar = new AStar(this->param.getXRows(), this->param.getYCols());
    this->astar = AStar::getInstance();

    this->initRobotsStates();
}

plotter::~plotter(){
    m_timer.stop();
}

//test astar algrithm
void plotter::initRobotsStates()
{
    Robot *r1 = new Robot(1,1,25,29, QString("ro"), ROBOT_gray);
    robotsArray.append(r1);
    Robot *r2 = new Robot(3,15,25,17, QString("ro2"), ROBOT_yellow);
    robotsArray.append(r2);
    Robot *r3 = new Robot(20,15,48,36, QString("ro3"), ROBOT_green);
    robotsArray.append(r3);
    Robot *r4 = new Robot(25,30,31,30, QString("ro4"), ROBOT_red);
    robotsArray.append(r4);
    Robot *r5 = new Robot(28,28,28,32, QString("ro5"), ROBOT_blue);
    robotsArray.append(r5);
}

//when use update, will call this fuction
void plotter::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setPen(Qt::black);
    drawAction(&painter);
}

void plotter::drawAction(QPainter *painter)
{
    drawMap(painter);

    //draw robot position and planned path.
    for(int i = 0; i < robotsArray.count(); i++){

        Robot *robo = robotsArray[i];

        drawRobotAndPath(robo, painter);

        /*******add cross planner********/
        QVector <posXY> posArray = robo->path;
        posXY pos = posArray[robo->robot_step];
        int tmp_pos_next = (robo->robot_step + 1 >= posArray.count() -1) ? posArray.count() -1 : robo->robot_step + 1;
        posXY pos_next = posArray[tmp_pos_next];
        int ind = param.posToIndex(pos);
        int ind_next = param.posToIndex(pos_next);

        robo->pose.index_now = ind;
        if(robo->robot_step == posArray.count() -1){ //path end
            // circle from the start
            robo->robot_step = 0;
        }else{
            // on the way
            if(astar->isIndexObstacle(ind_next)){
                //not to go to next step
            }else{
                robo->robot_step ++;
            }
        }

        astar->setIndexObstacle(ind_next); //add lock
        astar->setIndexNormal(ind);//free lock

    }

}

void plotter::drawMap(QPainter *painter)
{
    //if need dynamic , which will uncomment lines belows
    rect = QRect(margin, margin, width()-2*margin, height()-2*margin );
    xstep = (float)(rect.width())/ (this->XRows -1);
    ystep = (float)(rect.height())/ (this->YCols -1);

    /* draw x line */
    for(int i=0; i<this->XRows; i++)
    {
        int x = margin + i*xstep;
        painter->drawLine(x,rect.top(),x,rect.bottom());
    }

    /* draw y line */
    for(int j=0; j<this->YCols; j++)
    {
        int y = margin+(j*ystep);
        painter->drawLine(margin,y,rect.right(),y);
    }

    // show obstacles
    painter->setBrush(Qt::gray);
    painter->setPen(Qt::black);
    for(unsigned int i = 0; i < this->astar->obstacleIndexs.size(); i++){
        int index = this->astar->obstacleIndexs.at(i);
        posXY pos = this->param.indexToPos(index);
        int xx = (margin - robot_radius)  + pos.x * xstep ;
        int yy = (margin - robot_radius)  + pos.y * ystep ;
        painter->drawEllipse(xx, yy, robot_radius*2, robot_radius*2);
    }

    painter_times ++;
}

void plotter::drawRobotAndPath(Robot *robo, QPainter *painter)
{
    painter->setBrush(Qt::red);
    painter->setPen(pen);

    //draw robot planer path
    QVector <posXY> posArray = robo->path;
    for (int i = 0; i < posArray.count() -1; i++)
    {
        posXY pos = posArray[i] ;
        posXY pos1 = posArray[i+1] ;
        float x0 = margin + pos.x * xstep ;
        float y0= margin + pos.y * ystep ;
        float x1 = margin + pos1.x * xstep ;
        float y1= margin + pos1.y * ystep ;
        painter->drawLine(x0, y0, x1, y1) ;
    }

    //draw robots pos by pos(x,y)
    //std::cout << "robot_step :" << robo->robot_step << std::endl;
    posXY pos = posArray[robo->robot_step];
    switch (robo->robot_id) {
    case ROBOT_blue:
        painter->setBrush(Qt::blue);
        break;
    case ROBOT_red:
        painter->setBrush(Qt::red);
        break;
    case ROBOT_green:
        painter->setBrush(Qt::green);
        break;
    case ROBOT_gray:
        painter->setBrush(Qt::gray);
        break;
    case ROBOT_yellow:
        painter->setBrush(Qt::yellow);
        break;
    default:
        painter->setBrush(Qt::black);
        break;
    }
    painter->setPen(Qt::black) ;
    float x0 = (margin - robot_radius)  + pos.x * xstep ;
    float y0 = (margin - robot_radius) + pos.y * ystep ;
    painter->drawEllipse(x0, y0, robot_radius*2, robot_radius*2) ;
}

void plotter::timerEvent(QTimerEvent *event)
{
    Q_UNUSED(event);

    if (event->timerId() == m_timer.timerId())
    {
        update();//will call paintEvent
    }
    else
    {
        QWidget::timerEvent(event);
    }
}
