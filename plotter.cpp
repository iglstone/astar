#include "plotter.h"
#include <iostream>
#include <QtGlobal>

plotter::plotter(QWidget *parent) : QWidget(parent)
{
    this->XRows = 50;
    this->YCols = 40;
//    m_timer.start(500, this);
    m_timer.start(800, this);
    m_nStep = 0;

    margin = 35;//边缘
    rect = QRect(margin, margin, width()-2*margin, height()-2*margin );//取得绘图区域，大小要减去旁白
    xstep = (float)(rect.width())/ (this->XRows -1);
    ystep = (float)(rect.height())/ (this->YCols -1);

    //simulate a robot move forward
    rb.index_now = 12;
    rb.forward = ForWord_Front;

    //path pen style
    pen.setStyle(Qt::DotLine);
    pen.setWidth(3);
    pen.setBrush(Qt::red);

    robot_step = 0;

    this->initRobotsStates();
}

plotter::~plotter(){
    m_timer.stop();
}

//test astar algrithm
void plotter::initRobotsStates()
{
    astar = new AStar(this->getXRows(), this->getYCols());
    astar->Four_Neighbor = false;//if true, four neighbors search

    robot *rb_run = new robot;
    int ind_start = this->xyToIndex(1,1);
    int ind_end = this->xyToIndex(25,29);
    rb_run->index_start = ind_start;
    rb_run->index_end = ind_end;
    rb_run->robot_step = 0;
    QVector <posXY> paths = this->astarPathToMapPath(rb_run,astar);
    //rb_run.path = paths;
    rb_run->path.swap(paths);
    robotsArray.append(rb_run);

    robot *rb_run2 = new robot;
    int ind_start2 = this->xyToIndex(1,15);
    int ind_end2 = this->xyToIndex(40,35);
    rb_run2->index_start = ind_start2;
    rb_run2->index_end = ind_end2;
    rb_run2->robot_step = 0;
    QVector <posXY> paths2 = this->astarPathToMapPath(rb_run2,astar);
    rb_run2->path.swap(paths2);
    robotsArray.append(rb_run2);

}

QVector <posXY> plotter::astarPathToMapPath(robot *rob, AStar *astar){
    posXY p0 = this->indexToPos(rob->index_start);
    posXY p1 = this->indexToPos(rob->index_end);
    astar->startAStar(p0.x ,p0.y ,p1.x ,p1.y);

    //find path
    QVector <posXY> posArray;
    std::vector<std::pair<float, float>> path = astar->path;
    int count = path.size();
    for (int i = count -1 ; i >= 0; i--)
    {
        std::pair<float, float> pa = path[i];
        int x = pa.first ;
        int y = pa.second ;
        std::cout << "path x:" << x << "  y:" << y << std::endl ;
        posXY pos ;
        pos.x = x ;
        pos.y = y ;
        posArray.append(pos) ;
    }
    return posArray;
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
        robot *robo = robotsArray[i];

        painter->setBrush(Qt::red);
        painter->setPen(pen);

        QVector <posXY> posArray = robo->path;
        for (int i = 0; i < posArray.count() -1; i++)
        {
            posXY pos = posArray[i];
            posXY pos1 = posArray[i+1];
            float x0 = 35 + pos.x * xstep ;
            float y0= 35 + pos.y * ystep ;
            float x1 = 35 + pos1.x * xstep ;
            float y1= 35 + pos1.y * ystep ;
            painter->drawLine(x0, y0, x1, y1);
        }

        /* show the robot move */
        if(robo->robot_step < posArray.count()){
            std::cout << "robot_step :" << robo->robot_step << std::endl;
            posXY pos = posArray[robo->robot_step];
            painter->setBrush(Qt::blue);
            painter->setPen(Qt::black);
            float x0 = 30 + pos.x * xstep ;
            float y0= 30 + pos.y * ystep ;
            painter->drawEllipse(x0, y0, 10,10);
            if(robo->robot_step == posArray.count() -1){

            }else{
                std::cout << "xxxx :" << robo->robot_step << std::endl;
                robo->robot_step ++;
            }
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
    int x_o = 10;
    for(int y_o = 5; y_o < 30 ; y_o ++){
        int xx = 30 + x_o * xstep ;
        int yy = 30 + y_o * ystep ;
        painter->drawEllipse(xx, yy, 10,10);
    }







    /* show the robot */
    painter->setBrush(Qt::red);
    //examine map bundury
    bool bundery = false;
    posXY pos = this->indexToPos(rb.index_now);
    if(pos.x < 0){
        pos.x = 0;
        bundery = true;
    }
    if(pos.x >= (this->getXRows() -1)){
        pos.x = (this->getXRows() -1);
        bundery = true;
    }
    if(pos.y < 0){
        pos.y = 0;
        bundery = true;
    }
    if(pos.y >= (this->getYCols() -1)){
        pos.y = (this->getYCols() -1);
        bundery = true;
    }

    int index = this->posToIndex(pos);
    rb.index_now = index ;
    int x = 30 + pos.x * xstep ;
    int y = 30 + pos.y * ystep ;

    painter->drawEllipse(x, y, 10,10);

    if(bundery){
        return;
    }

    /* move the robot */
    switch (rb.forward) {
        case ForWord_Front:
            rb.index_now++;
            break;
        case ForWord_Back:
            rb.index_now--;
            break;
        case ForWord_Down:
            rb.index_now += this->getXRows();
            break;
        case ForWord_Up:
            rb.index_now -= this->getXRows();
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

posXY plotter::indexToPos(int index){
    posXY pos ;
    if (index >= (this->getXRows() ) * (this->getYCols() ) || index < 0 ){
        printf("index out of the bundrary! sth error!\n");
    }else{
        int x = index % (this->getXRows());
        int y = index / this->getXRows();

        pos.x = x;
        pos.y = y;
    }
    //std::cout << "pos x:" << pos.x << "  pos y:" << pos.y <<std::endl;
    return pos;
}

int plotter::xyToIndex(int x, int y){
    int index = y * this-> getXRows() + x;
    return index;
}

int plotter::posToIndex(posXY pos){
    int x = pos.x;
    int y = pos.y;
    int index = this->xyToIndex(x,y);
    return index;
}

int plotter::getXRows(){
    return this->XRows ;
}
int plotter::getYCols(){
    return this->YCols;
}
void plotter::setXRows(int row){
    this->XRows = row;
}
void plotter::setYCols(int col){
    this->YCols = col;
}
