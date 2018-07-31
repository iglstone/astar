#include "Robot.h"
#include <QTime>
#include <QDebug>
#include <sys/time.h>

Robot::Robot(QObject *parent) : QObject(parent)
{
    this->init(0,0);
}

Robot::Robot(int start_x, int start_y, int end_x, int end_y, QString name, int id)
{
    this->init(start_x, start_y);
    this->name = name;
    this->robot_id = id;
    this->index_start = this->param.xyToIndex(start_x, start_y);
    this->index_end = this->param.xyToIndex(end_x, end_y);

    QVector <posXY> paths = this->astarPathToMapPath(this->index_start, this->index_end);
    this->path.swap(paths);
}

// tag coord
Robot::Robot(int start_x, int start_y)
{
    this->init(start_x, start_y);
    QVector <posXY> paths = this->astarPathToMapPath(this->index_start, this->index_end);
    this->path.swap(paths);
}

void Robot::init(int start_x, int start_y){
    RobotPose pose ;
    pose.forward = ForWord_Self;
    pose.index_now = param.xyToIndex(start_x, start_y);
    pose.x_coord_tag = 0.0;
    pose.y_coord_tag = 0.0;

    this->pose = pose;
    this->name = "no name"; //TODO: id to string
    this->robot_id = 0; //need random
    this->robot_step = 0;
    this->index_start = 0;
    this->index_end = 0;

    astar = AStar::getInstance();
}

//Robot::~Robot(){
//    //delete astar;
//}

QVector <posXY> Robot::astarPathToMapPath(int index_start, int index_end)
{
    posXY p0 = this->param.indexToPos(index_start);
    posXY p1 = this->param.indexToPos(index_end);

    //QTime time;
    //time.start();
    struct timeval tpstart,tpend;
    float timeuse;
    gettimeofday(&tpstart,NULL);
    this->astar->startAStar(p0.x ,p0.y ,p1.x ,p1.y);

    //qDebug()<<"astar :"<<time.elapsed()<<"ms";
    //find path

    QVector <posXY> posArray;
    std::vector<std::pair<float, float> > path = astar->path;

    gettimeofday(&tpend,NULL);
    timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec);
    qDebug()<<"time use:" << timeuse<<"us";

    int count = path.size();
    for (int i = count -1 ; i >= 0; i--)
    {
        std::pair<float, float> pa = path[i];
        int x = pa.first;
        int y = pa.second;
        //std::cout << "path x:" << x << "  y:" << y << std::endl ;
        posXY pos;
        pos.x = x;
        pos.y = y;
        posArray.append(pos);
    }

    return posArray;
}

