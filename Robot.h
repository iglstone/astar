#ifndef ROBOT_H
#define ROBOT_H

#include <QObject>
#include "Parameters.h"
#include <QVector>
#include "astar.h"

enum ROBOT_ForWord{
    ForWord_Self    = 0,
    ForWord_Front   = 1,
    ForWord_Up      = 2,
    ForWord_Back    = 3,
    ForWord_Down    = 4,
    ForWord_Unknow  = 5
};

enum ROBOT_Color{
    ROBOT_black    = 0,
    ROBOT_yellow   = 1,
    ROBOT_gray     = 2,
    ROBOT_green    = 3,
    ROBOT_red      = 4,
    ROBOT_blue     = 5
};

struct RobotPose{
    int index_now;//now position tag
    float x_coord_tag;// from odom
    float y_coord_tag;// from odom
    ROBOT_ForWord forward; //朝向, not use now

    //float x_coord_map;// index_pose + odom
    //float y_coord_map;
};


class Robot : public QObject
{
    Q_OBJECT
public:
    explicit Robot(QObject *parent = 0);
//    Robot();
//    ~Robot();
    Robot(int start_x, int start_y);
//    Robot * initARobot(int start_x, int start_y, int end_x, int end_y, QString name, int id);
    Robot(int start_x, int start_y, int end_x, int end_y, QString name, int id);

    RobotPose pose;
    QString name;
    int robot_id;
    int index_start; // for nav start
    int index_end; // for nav end
    int robot_step; //means robot speed
    QVector <posXY> path; //astar planed path

signals:

private:
    Parameters param ;
    AStar *astar;
    QVector <posXY> astarPathToMapPath(int index_start, int index_end);
    void init(int start_x, int start_y);

public slots:
};

#endif // ROBOT_H
