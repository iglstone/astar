#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <QObject>

#define MAP_X 60 //x coor sum
#define MAP_Y 40 //y coor sum

struct posXY{
    int x;
    int y;
};

class Parameters : public QObject
{
    Q_OBJECT
public:
    explicit Parameters(QObject *parent = 0);
    //    Parameters();
    posXY indexToPos(int index);
    int xyToIndex(int x, int y);
    int posToIndex(posXY pos);
    int getXRows();
    int getYCols();
    void setXRows(int row);
    void setYCols(int col);

signals:

public slots:

private:
    int row_;
    int col_;
};

#endif // PARAMETERS_H
