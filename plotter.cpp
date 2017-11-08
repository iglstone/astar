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
    astar->Four_Neighbor = true;//if true, four neighbors search

    robot *ro = this->initARobot(1,1,25,29, QString("ro"), 1);
    robotsArray.append(ro);

    robot *ro2 = this->initARobot(1,15,25,17, QString("ro2"), 2);
    robotsArray.append(ro2);

    robot *ro3 = this->initARobot(20,15,48,36, QString("ro3"), 3);
    robotsArray.append(ro3);

    //ro4 is before ro5
    robot *ro4 = this->initARobot(25,30,31,30, QString("ro4"), 4);
    robotsArray.append(ro4);

    robot *ro5 = this->initARobot(28,28,28,32, QString("ro5"), 5);
    robotsArray.append(ro5);
}

robot * plotter::initARobot(int start_x, int start_y, int end_x, int end_y, QString name, int id){
    robot *r = new robot;//if use ptr, must new one
    int ind_start = this->xyToIndex(start_x,start_y);
    int ind_end = this->xyToIndex(end_x,end_y);
    r->index_start = ind_start;
    r->index_end = ind_end;
    r->robot_step = 0;
    r->name = name;
    r->robot_id = id;
    QVector <posXY> paths = this->astarPathToMapPath(r,astar);
    //rb_run.path = paths;
    r->path.swap(paths);
    return r;
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
            case 5:
                painter->setBrush(Qt::blue);
                break;
            case 4:
                painter->setBrush(Qt::red);
                break;
            case 3:
                painter->setBrush(Qt::green);
                break;
            case 2:
                painter->setBrush(Qt::gray);
                break;
            case 1:
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


            /*******add now********/
            int ind = this->posToIndex(pos);
            robo->index_now = ind;
            //if next one is the final, stop; else point to the next one
            int tmp_next = (robo->robot_step + 1 >= posArray.count() -1) ? posArray.count() -1 : robo->robot_step + 1;

            posXY pos_next = posArray[tmp_next];
            int ind_next = this->posToIndex(pos_next);

            if(robo->robot_step == posArray.count() -1){ //means stop
                // circle from the start
                robo->robot_step = 0;
            }else{ //means start to move
                if(astar->isIndexObstacle(ind_next)){

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
        posXY pos = this->indexToPos(index);
        int xx = 30 + pos.x * xstep ;
        int yy = 30 + pos.y * ystep ;
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
