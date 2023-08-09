#include "mainwindow.hpp"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->stackedWidget->insertWidget(1, &_pid_controller);
    ui->stackedWidget->insertWidget(2, &_mission_planner);

//    connect(&_pid_controller, SIGNAL(HomeClicked()), this, SLOT(moveHome()));
//    connect(&_mission_planner, SIGNAL(HomeClicked()), this, SLOT(moveHome()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pid_controller_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);

}

void MainWindow::on_mission_planning_clicked()
{
        ui->stackedWidget->setCurrentIndex(2);
}

//void MainWindow::moveHome()
//{
//        ui->stackedWidget->setCurrentIndex(0);
//}

void MainWindow::on_homeButton_clicked()
{
    ui->stackedWidget->setCurrentIndex(0);
}

// void executeTerminalCommand(const char* command) {
//     int result = std::system(command);
    
//     if (result == 0) {
//         std::cout << "Command executed successfully." << std::endl;
//     } else {
//         std::cerr << "Command execution failed." << std::endl;
//     }
// }

// void MainWindow::on_visionButton_clicked()
// {
//     executeTerminalCommand("python3 ~/master/scripts/vision/make_dataset.py");
// }

