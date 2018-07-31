#include "Parameters.h"
#include "stdio.h"

Parameters::Parameters(QObject *parent) : QObject(parent)
{
    this->row_ = MAP_X;
    this->col_ = MAP_Y;
}

posXY Parameters::indexToPos(int index){
    posXY pos ;
    if (index >= (this->getXRows() ) * (this->getYCols() ) || index < 0 ){
        printf("index out of the bundrary! sth error!\n");
    }else{
        int x = index % (this->getXRows());
        int y = index / this->getXRows();

        pos.x = x;
        pos.y = y;
    }

    return pos;
}


int Parameters::xyToIndex(int x, int y){
    int index = y * this-> getXRows() + x;
    return index;
}

int Parameters::posToIndex(posXY pos){
    int x = pos.x;
    int y = pos.y;
    int index = this->xyToIndex(x,y);
    return index;
}

int Parameters::getXRows(){
    return this->row_ ;
}
int Parameters::getYCols(){
    return this->col_;
}
void Parameters::setXRows(int row){
    this->row_ = row;
}
void Parameters::setYCols(int col){
    this->col_ = col;
}
