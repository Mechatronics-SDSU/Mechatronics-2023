#include "mainwindow.hpp"
#include "./ui_mainwindow.h"
#include <iostream>
#include <QTimer>
#include <cstring>
#include <QComboBox>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    this->node = rclcpp::Node::make_shared("gui_node");
    this->kp_publisher = node->create_publisher<scion_types::msg::PidTuning>("kp_dial_data", 10);
    this->ki_publisher = node->create_publisher<scion_types::msg::PidTuning>("ki_dial_data", 10);
    this->kd_publisher = node->create_publisher<scion_types::msg::PidTuning>("kd_dial_data", 10);
    ui->setupUi(this);
    QObject::connect(ui->Kp, &QDial::valueChanged, this, &MainWindow::updateKpValue);
    QObject::connect(ui->Ki, &QDial::valueChanged, this, &MainWindow::updateKiValue);
    QObject::connect(ui->Kd, &QDial::valueChanged, this, &MainWindow::updateKdValue);
    QObject::connect(ui->comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::handleIndexChanged);    
}

void MainWindow::handleIndexChanged()
{
    this->axis = ui->comboBox->currentIndex();
}

void MainWindow::updateKpValue()
{
    auto message = scion_types::msg::PidTuning();
    message.data = ui->Kp->value();
    message.axis = this->axis;
    kp_publisher->publish(message);
}

void MainWindow::updateKiValue()
{
    auto message = scion_types::msg::PidTuning();
    message.data = ui->Ki->value();
    message.axis = this->axis;
    ki_publisher->publish(message);
}

void MainWindow::updateKdValue()
{
    auto message = scion_types::msg::PidTuning();
    message.data = ui->Kd->value();
    message.axis = this->axis;
    kd_publisher->publish(message);
}

MainWindow::~MainWindow()
{
    delete ui;
}

