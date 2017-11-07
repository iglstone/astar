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
    //forward = qrand() % 5;
    rb.index = 12;
    rb.forward = ForWord_Front;

    this->testAStar();
}

plotter::~plotter(){
    m_timer.stop();
}

//test astar algrithm
void plotter::testAStar()
{
    astar = new AStar(this->getXRows(), this->getYCols());
    astar->Four_Neighbor = false;//if true, four neighbors search
    astar->startAStar(1 ,1 ,25 ,29);

    //find path
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

    /* show path */
    painter->setBrush(Qt::red);
    for (int i = 0; i < posArray.count(); i++)
    {
        posXY pos = posArray[i];
        float x = 30 + pos.x * xstep ;
        float y = 30 + pos.y * ystep ;
        painter->drawEllipse(x, y, 10,10);
    }

    //show the blink robot
    int nIndex = (m_nStep) % 2;
    if(nIndex){
        painter->setBrush(Qt::blue);
        painter->drawEllipse(30, 30, 10,10);
    }else{

    }

    /* show obstacles */
    painter->setBrush(Qt::gray);
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
    posXY pos = this->indexToPos(rb.index);
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
    rb.index = index ;
    int x = 30 + pos.x * xstep ;
    int y = 30 + pos.y * ystep ;

    painter->drawEllipse(x, y, 10,10);

    if(bundery){
        return;
    }

    /* move the robot */
    switch (rb.forward) {
        case ForWord_Front:
            rb.index++;
            break;
        case ForWord_Back:
            rb.index--;
            break;
        case ForWord_Down:
            rb.index += this->getXRows();
            break;
        case ForWord_Up:
            rb.index -= this->getXRows();
            break;
        case ForWord_Self:
            std::cout << " get the goal" << std::endl;
            break;
        default:
            break;
    }
}

void plotter::drawCircle(int index){
    posXY pos = this->indexToPos(index);
    posArray.append(pos);
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
    std::cout << "pos x:" << pos.x << "  pos y:" << pos.y <<std::endl;
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
