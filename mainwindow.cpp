#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QHBoxLayout"
#include "QPushButton"
#include <iostream>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->plot = new plotter;
    this->plot->resize(500, 400);

//    QHBoxLayout *topLayout = new QHBoxLayout;
//    QPushButton *btn_ok = new QPushButton(QWidget::tr("OK"), this);
//    connect(btn_ok,SIGNAL(clicked()),this,SLOT(on_pushButton_clicked()));
//    topLayout->addWidget(this->plot);
//    topLayout->addWidget(btn_ok);
//    this->setLayout(topLayout);

    this->resize(1000, 600);

    this->setCentralWidget(this->plot);
    num = 1;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked(){
    //std::cout << "xxx" << std::endl;
    //printf("xxx ooo");

    //    this->plot->drawCircle(num);
    //    num++;

    //this->plot->robot_step = 0;//sth wrong here
}

void MainWindow::on_setARobot_clicked()
{

}
