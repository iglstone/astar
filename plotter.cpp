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
    m_nStep = 0;

    margin = 35;//边缘
    rect = QRect(margin, margin, this->width()-2*margin, this->height()-2*margin );//取得绘图区域，大小要减去旁白
    xstep = (float)(rect.width())/ (this->XRows -1);
    ystep = (float)(rect.height())/ (this->YCols -1);

    rb.pose.index_now = 12;
    rb.pose.forward= ForWord_Front;

    //path pen style
    pen.setStyle(Qt::DotLine);
    pen.setWidth(3);
    pen.setBrush(Qt::red);

    robot_step = 0;

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

void plotter::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    drawGrid(&painter);
}

void plotter::drawGrid(QPainter *painter)
{
    painter->setPen(Qt::black);

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

    /* show astar path */
    for(int i = 0; i < robotsArray.count(); i++){
        Robot *robo = robotsArray[i];

        painter->setBrush(Qt::red);
        painter->setPen(pen);

        QVector <posXY> posArray = robo->path;
        for (int i = 0; i < posArray.count() -1; i++)
        {
            posXY pos = posArray[i] ;
            posXY pos1 = posArray[i+1] ;
            float x0 = 35 + pos.x * xstep ;
            float y0= 35 + pos.y * ystep ;
            float x1 = 35 + pos1.x * xstep ;
            float y1= 35 + pos1.y * ystep ;
            painter->drawLine(x0, y0, x1, y1) ;
        }

        /* show the robot move */
        if(robo->robot_step < posArray.count()){
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
            painter->setPen(Qt::black);
            float x0 = 30 + pos.x * xstep ;
            float y0= 30 + pos.y * ystep ;
            painter->drawEllipse(x0, y0, 10,10);


            /*******add cross planner********/
            int ind = param.posToIndex(pos);
            robo->pose.index_now = ind;
            //if next one is the final, stop; else point to the next one
            int tmp_next = (robo->robot_step + 1 >= posArray.count() -1) ? posArray.count() -1 : robo->robot_step + 1;

            posXY pos_next = posArray[tmp_next];
            int ind_next = param.posToIndex(pos_next);

            if(robo->robot_step == posArray.count() -1){ //means stop
                // circle from the start
                robo->robot_step = 0;
            }else{
                //means start to move
                if(astar->isIndexObstacle(ind_next)){
                    //do nothing
                }else{
                    robo->robot_step ++;
                }
            }
            astar->setIndexObstacle(ind_next);
            astar->setIndexNormal(ind);//free the space
        }
    }

    //show the blink robot on 0.0
    int nIndex = (m_nStep) % 2;
    if(nIndex){
        painter->setBrush(Qt::blue);
        painter->drawEllipse(30, 30, 10,10);
    }else{

    }

    /* show obstacles */
    painter->setBrush(Qt::gray);
    painter->setPen(Qt::black);
    for(unsigned int i = 0; i < this->astar->obstacleIndexs.size(); i++){
        int index = this->astar->obstacleIndexs.at(i);
        posXY pos = this->param.indexToPos(index);
        int xx = 30 + pos.x * xstep ;
        int yy = 30 + pos.y * ystep ;
        painter->drawEllipse(xx, yy, 10,10);
    }


    /* show the right forward blink robot */
    painter->setBrush(Qt::red);
    //examine map bundury
    bool bundery = false;
    posXY pos = this->param.indexToPos(rb.pose.index_now);
    if(pos.x < 0){
        pos.x = 0;
        bundery = true;
    }
    if(pos.x >= (this->param.getXRows() -1)){
        pos.x = (this->param.getXRows() -1);
        bundery = true;
    }
    if(pos.y < 0){
        pos.y = 0;
        bundery = true;
    }
    if(pos.y >= (this->param.getYCols() -1)){
        pos.y = (this->param.getYCols() -1);
        bundery = true;
    }

    int index = this->param.posToIndex(pos);
    rb.pose.index_now = index ;
    int x = 30 + pos.x * xstep ;
    int y = 30 + pos.y * ystep ;

    painter->drawEllipse(x, y, 10,10);

    if(bundery){
        return;
    }

    /* move the robot */
    switch (rb.pose.forward) {
        case ForWord_Front:
            rb.pose.index_now++;
            break;
        case ForWord_Back:
            rb.pose.index_now--;
            break;
        case ForWord_Down:
            rb.pose.index_now += this->param.getXRows();
            break;
        case ForWord_Up:
            rb.pose.index_now -= this->param.getXRows();
            break;
        case ForWord_Self:
            std::cout << " get the goal" << std::endl;
            break;
        default:
            break;
    }


}

void plotter::timerEvent(QTimerEvent *event)
{
    Q_UNUSED(event);

    if (event->timerId() == m_timer.timerId())
    {
        ++m_nStep;
        update();
    }
    else
    {
        QWidget::timerEvent(event);
    }
}
