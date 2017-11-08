#ifndef ASTAR_H
#define ASTAR_H

#include <QObject>
#include <vector>
#include <algorithm>

#define POT_HIGH 1.0e10        // unassigned cell potential

namespace costmap
{
static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char FREE_SPACE = 0;
static const unsigned char NORMAL_SPACE = 50;
}

class Index {
    public:
        Index(int a, float b) {
            i = a;
            cost = b;
        }
        int i;
        float cost;
};

struct greater1 {
        bool operator()(const Index& a, const Index& b) const {
            return a.cost > b.cost;
        }
};

struct posXY{
    int x;
    int y;
};

class AStar : public QObject
{
    Q_OBJECT
public:
    explicit AStar(QObject *parent = 0);
    AStar(int nx, int ny);
    std::vector< std::pair<float, float> > path;
    bool Four_Neighbor; // if false, means 8-neighbor
    bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                             int cycles, float* potential);
    bool getPath(float* potential, double start_x, double start_y, double end_x,
                        double end_y, std::vector<std::pair<float, float> >& path);
    void startAStar(int start_x, int start_y, int goal_x, int goal_y);
    std::vector<int> obstacleIndexs;
    void setIndexObstacle(int index);
    void setIndexNormal(int index);
    bool isIndexObstacle(int index);

signals:

public slots:

private:
    void add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x, int end_y);
    float calculatePotential(float* potential, unsigned char cost, int n, float prev_potential);
    std::vector<Index> queue_;
    int getIndex(int x, int y);
    void outlineMap(unsigned char* costarr, unsigned char value);
    posXY indexToPos(int index);

    int nx_, ny_, ns_; /**< size of grid, in pixels */
    bool unknown_;
    unsigned char lethal_cost_, neutral_cost_;
    int cells_visited_;
    float * potential_array_ ;
    unsigned char* costmap_;

    void setIndexsObstacles(std::vector<int> indexs);

};

#endif // ASTAR_H
