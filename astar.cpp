#include "astar.h"
#include "iostream"

QMutex AStar::mutex;
AStar *AStar::astar_instance = NULL;

AStar::AStar(QObject *parent) : QObject(parent)
{

}

AStar *AStar::getInstance(){
    if(astar_instance == NULL){
        mutex.lock();
        if(NULL == astar_instance){
            astar_instance = new AStar(MAP_X, MAP_Y);
        }
        mutex.unlock();
    }
    return astar_instance;
}

AStar::AStar(int nx, int ny){

    std::cout << "..init astar.." << std::endl;

    nx_ = nx;
    ny_ = ny;
    ns_ = nx * ny;

    unknown_ = true;
    lethal_cost_ = 253;
    neutral_cost_ = 50;//经验值

    potential_array_ = new float[nx * ny];

    unsigned char cost = 50;
    costmap_ = (unsigned char *)malloc(ns_);
    for(int i = 0; i < ns_; i++){
        costmap_[i] = cost;
    }

    //set obstacles
    int x = 10;
    for(int y = 5; y < 30 ; y ++)
    {
        if(y == 10 || y == 16 || y == 24){
            continue;
        }
        int index = this->getIndex(x,y);
        obstacleIndexs.push_back(index);
    }
    this->setIndexsObstacles(obstacleIndexs);
}

void AStar::setIndexsObstacles(std::vector<int> indexs)
{
    for(unsigned int i = 0; i < indexs.size(); i++){
        int index = indexs.at(i);
        costmap_[index] = costmap::INSCRIBED_INFLATED_OBSTACLE;
    }
}

void AStar::setIndexObstacle(int index)
{
    if (index >= nx_ * ny_ || index < 0 ){
        std::cout << "index out of the bundrary! error!\n" << std::endl;
    }else{
        costmap_[index] = costmap::INSCRIBED_INFLATED_OBSTACLE;
    }
}

bool AStar::isIndexObstacle(int index)
{
    if (index >= nx_ * ny_ || index < 0 ){
        std::cout << "index out of the bundrary! error!\n" << std::endl;;
        return true;
    }else{
        int cost = costmap_[index] ;
        //std::cout << "costs :" << cost << " index :" << index << std::endl;
        if(cost >= costmap::INSCRIBED_INFLATED_OBSTACLE){
            return true;
        }
        return false;
    }
}

void AStar::setIndexNormal(int index){
    if (index >= nx_ * ny_ || index < 0 ){
        std::cout << "index out of the bundrary! error!\n" << std::endl;
    }else{
        costmap_[index] = costmap::NORMAL_SPACE;
    }
}

void AStar::startAStar(int start_x, int start_y, int goal_x, int goal_y){
    this->outlineMap(costmap_, costmap::LETHAL_OBSTACLE);
    bool found_legal = this->calculatePotentials(costmap_, start_x, start_y,
                                                 goal_x, goal_y, nx_ * ny_ * 2, potential_array_);
    if (found_legal){
        std::cout << "found the path" << std::endl;
        this->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path);
    }
}

bool AStar::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                int cycles, float* potential)
{
    queue_.clear();
    int start_i = this->getIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));

    std::fill(potential, potential + ns_, POT_HIGH);
    potential[start_i] = 0;

    int goal_i = this->getIndex(end_x, end_y);
    int cycle = 0;

    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        queue_.pop_back();

        int i = top.i;
        if (i == goal_i)
            return true;

        add(costs, potential, potential[i], i + 1, end_x, end_y);
        add(costs, potential, potential[i], i - 1, end_x, end_y);
        add(costs, potential, potential[i], i + nx_, end_x, end_y);
        add(costs, potential, potential[i], i - nx_, end_x, end_y);

        cycle++;
    }

    return false;
}

void AStar::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x, int end_y)
{
    if (next_i < 0 || next_i >= ns_)
        return;

    if (potential[next_i] < POT_HIGH) //it means the potential cell has been explored
        return;

    if(costs[next_i] >= lethal_cost_ && !(unknown_ && costs[next_i]==costmap::NO_INFORMATION)) //it means this cell is obstacle
        return;

    // compute the next_i cell in potential
    // costs[next_i] + neutral_cost_:original cost + extra cost
    // potential[next_i] means the cost from start to current
    potential[next_i] = this->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);

    int x = next_i % nx_, y = next_i / nx_;
    float distance = abs(end_x - x) + abs(end_y - y);
    //distance * neutral_cost_ means the current to the target
    //f(n)=g(n)+h(n)
    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
    std::push_heap(queue_.begin(), queue_.end(), greater1());//sort about the previous insert element

}

float AStar::calculatePotential(float* potential, unsigned char cost, int next_i, float prev_potential=-1)
{
    if(prev_potential < 0){
        // get min of neighbors
        float min_h = std::min( potential[next_i - 1], potential[next_i + 1] );
        float min_v = std::min( potential[next_i - nx_], potential[next_i + nx_]);
        prev_potential = std::min(min_h, min_v);
    }

    return prev_potential + cost;
}


bool AStar::getPath(float* potential, double start_x, double start_y, double end_x, double end_y,
                    std::vector< std::pair<float, float> >& path_)
{
    path_.clear();
    std::pair<float, float> current;
    current.first = end_x;
    current.second = end_y;

    int start_index = getIndex(start_x, start_y);

    path_.push_back(current);
    int c = 0;
    int ns = nx_ * ny_;

    while (getIndex(current.first, current.second) != start_index) {
        float min_val = 1e10;
        int min_x = 0, min_y = 0;
        for (int xd = -1; xd <= 1; xd++) {
            for (int yd = -1; yd <= 1; yd++) {
                if (xd == 0 && yd == 0)
                    continue;

                if(this->Four_Neighbor){
                    if(xd * yd == 1 || xd * yd == -1){
                        continue;
                    }
                }

                int x = current.first + xd;
                int y = current.second + yd;
                int index = getIndex(x, y);
                if (potential[index] < min_val) {
                    min_val = potential[index];
                    min_x = x;
                    min_y = y;
                }
            }
        }
        if (min_x == 0 && min_y == 0)
            return false;
        current.first = min_x;
        current.second = min_y;
        path_.push_back(current);

        if(c++>ns*4){
            return false;
        }
    }
    return true;
}

//set the bundary of the map obstacle
void AStar::outlineMap(unsigned char* costarr, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx_; i++)
        *pc++ = value;
    pc = costarr + (ny_ - 1) * nx_;
    for (int i = 0; i < nx_; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny_; i++, pc += nx_)
        *pc = value;
    pc = costarr + nx_ - 1;
    for (int i = 0; i < ny_; i++, pc += nx_)
        *pc = value;
}

int AStar::getIndex(int x, int y) {
    return x + y * nx_;
}

posXY AStar::indexToPos(int index){
    posXY pos ;
    if (index >= nx_ * ny_ || index < 0 ){
        std::cout << "index out of the bundrary! sth error!\n" << std::endl;
    }else{
        int x = index % nx_;
        int y = index / nx_;

        pos.x = x;
        pos.y = y;
    }
    return pos;
}
