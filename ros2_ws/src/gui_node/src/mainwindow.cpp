#include "mainwindow.hpp"
#include "./ui_mainwindow.h"
#include <iostream>
#include <QTimer>
#include <cstring>
#include <QComboBox>

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
    QObject::connect(ui->comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::handleIndexChanged);    
}

void MainWindow::handleIndexChanged(){
    this->axis = ui->comboBox->currentIndex();
    RCLCPP_INFO(this->node->get_logger(), "%d", this->axis);
}

void MainWindow::updateKpValue()
{
    auto message = std_msgs::msg::Int32();
    message.data = ui->Kp->value();
    // message.data = fmt::format("Action index: {}, Published: {}", this->action, ui->Kp->value());
    kp_publisher->publish(message);
}

void MainWindow::updateKiValue()
{
    auto message = std_msgs::msg::Int32();
    message.data = ui->Ki->value();
    // message.data = fmt::format("Action index: {}, Published: {}", this->action, ui->Ki->value());
    ki_publisher->publish(message);
}

void MainWindow::updateKdValue()
{
    auto message = std_msgs::msg::Int32();
    message.data = ui->Kd->value();
    // message.data = fmt::format("Action index: {}, Published: {}", this->action, ui->Kd->value());
    kd_publisher->publish(message);
}

MainWindow::~MainWindow()
{
    delete ui;
}

