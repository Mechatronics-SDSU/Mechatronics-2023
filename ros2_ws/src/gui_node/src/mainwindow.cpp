#include "mainwindow.hpp"
#include "./ui_mainwindow.h"
#include <iostream>
#include <QTimer>
#include <cstring>
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QObject::connect(ui->Kp, &QDial::valueChanged, this, &MainWindow::updateDialValue);

    // ui->Kp->
}

void MainWindow::updateDialValue()
{
    int value = ui->Kp->value();
    ui->pingLabel->setText(QString::fromStdString(std::to_string(value)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

