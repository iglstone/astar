#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "plotter.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

signals:
   void clicked();

//private slots:
//    void on_setARobot_clicked();
//    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    plotter *plot;
    int num;
};

#endif // MAINWINDOW_H
