#include "mainwindow.hpp"
#include "./ui_mainwindow.h"
#include <iostream>
#include <QTimer>
#include <cstring>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    this->node = rclcpp::Node::make_shared("gui_node");
    this->kp_publisher = node->create_publisher<std_msgs::msg::Int32>("kp_dial_data", 10);
    this->ki_publisher = node->create_publisher<std_msgs::msg::Int32>("ki_dial_data", 10);
    this->kd_publisher = node->create_publisher<std_msgs::msg::Int32>("kd_dial_data", 10);

    ui->setupUi(this);
    QObject::connect(ui->Kp, &QDial::valueChanged, this, &MainWindow::updateKpValue);
    QObject::connect(ui->Ki, &QDial::valueChanged, this, &MainWindow::updateKiValue);
    QObject::connect(ui->Kd, &QDial::valueChanged, this, &MainWindow::updateKdValue);
}

void MainWindow::updateKpValue()
{
    auto message = std_msgs::msg::Int32();
    message.data = ui->Kp->value();;
    kp_publisher->publish(message);
}

void MainWindow::updateKiValue()
{
    auto message = std_msgs::msg::Int32();
    message.data = ui->Ki->value();;
    ki_publisher->publish(message);
}

void MainWindow::updateKdValue()
{
    auto message = std_msgs::msg::Int32();
    message.data = ui->Kd->value();;
    kd_publisher->publish(message);
}

MainWindow::~MainWindow()
{
    delete ui;
}

