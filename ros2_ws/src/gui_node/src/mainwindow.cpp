#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <unistd.h>

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

    QObject::connect(ui->pingButton,    &QCommandLinkButton::clicked, this, &MainWindow::pingButtonClicked);
    QObject::connect(ui->rosButton,     &QCommandLinkButton::clicked, this, &MainWindow::rosButtonClicked);
    QObject::connect(ui->launchButton,  &QCommandLinkButton::clicked, this, &MainWindow::launchButtonClicked);
    // QObject::connect(ui->Kp, &QDial::valueChanged, this, &MainWindow::updateKpValue);
    // QObject::connect(ui->Ki, &QDial::valueChanged, this, &MainWindow::updateKiValue);
    // QObject::connect(ui->Kd, &QDial::valueChanged, this, &MainWindow::updateKdValue);
    QObject::connect(ui->Kp_push_button,  &QPushButton::clicked, this, &MainWindow::kpPushButtonClicked);
    QObject::connect(ui->Ki_push_button,  &QPushButton::clicked, this, &MainWindow::kiPushButtonClicked);
    QObject::connect(ui->Kd_push_button,  &QPushButton::clicked, this, &MainWindow::kdPushButtonClicked);
    QObject::connect(ui->comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::handleIndexChanged); 

}

void MainWindow::handleIndexChanged()
{
    this->axis = ui->comboBox->currentIndex();
}

void MainWindow::executeCommand(const char* command, QLabel* label)
{
    FILE* pipe = popen(command, "r");
    char buffer[128];
    fgets(buffer, sizeof(buffer), pipe);
    label->setText(buffer);
    sleep(6);
    pclose(pipe);
}

void MainWindow::launchButtonClicked()
{
    executeCommand("ros2 launch src/launch.py", ui->launchLabel);
}
void MainWindow::pingButtonClicked()
{
    executeCommand("ros2 launch src/launch.py", ui->pingLabel);
}
void MainWindow::rosButtonClicked()
{
    executeCommand("ros2 topic list", ui->rosLabel);
}

// void MainWindow::updateKpValue()
// {
//     auto message = scion_types::msg::PidTuning();
//     message.data = ui->Kp->value();
//     message.axis = this->axis;
//     kp_publisher->publish(message);
// }

// void MainWindow::updateKiValue()
// {
//     auto message = scion_types::msg::PidTuning();
//     message.data = ui->Ki->value();
//     message.axis = this->axis;
//     ki_publisher->publish(message);
// }

// void MainWindow::updateKdValue()
// {
//     auto message = scion_types::msg::PidTuning();
//     message.data = ui->Kd->value();
//     message.axis = this->axis;
//     kd_publisher->publish(message);
// }

// Halie's new code

void MainWindow::kpPushButtonClicked()
{
    
    this->kpVal = ui->Kp_value->text().toFloat();

    auto message = scion_types::msg::PidTuning();
    message.data = this->kpVal;
    message.axis = this->axis;
    kp_publisher->publish(message);
}
void MainWindow::kiPushButtonClicked()
{
    
    this->kiVal = ui->Ki_value->text().toFloat();

    auto message = scion_types::msg::PidTuning();
    message.data = this->kiVal;
    message.axis = this->axis;
    ki_publisher->publish(message);
}
void MainWindow::kdPushButtonClicked()
{
    
    this->kdVal = ui->Kd_value->text().toFloat();

    auto message = scion_types::msg::PidTuning();
    message.data = this->kdVal;
    message.axis = this->axis;
    kd_publisher->publish(message);
}


MainWindow::~MainWindow()
{
    delete ui;
}

