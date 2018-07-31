#include "mainwindow.h"
#include <QApplication>
#include <sys/time.h>
#include <QDebug>

void function(){
    int i = 0;
    while(i < 1000000)i++;
}

void calcByGetTimeOfDay() {
    struct timeval tpstart,tpend;
    float timeuse;

    gettimeofday(&tpstart,NULL);
    function();
    gettimeofday(&tpend,NULL);
    timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;

    qDebug()<<timeuse<<"s";
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    //calcByGetTimeOfDay();

    w.show();

    return a.exec();
}
