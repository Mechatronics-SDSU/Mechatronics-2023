#include "mainwindow.hpp"
#include "./ui_mainwindow.h"
#include <iostream>
#include <nlohmann/json.hpp>
#include <QString>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->stackedWidget->insertWidget(1, &_pid_controller);
    ui->stackedWidget->insertWidget(2, &_mission_planner);
    ui->nodes_to_enable->setReadOnly(true);
    this->json_string["nodes_to_enable"] = this->jsonArray;

    QString styleSheet = "color: #607cff; background-color: #242526;";
    ui->nodes_to_enable->setStyleSheet(styleSheet);
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


void MainWindow::on_brain_toggled(bool checked)
{
    if (checked){
        this->jsonArray.push_back(ui->brain->text().toStdString());
    } else {
        this->jsonArray.erase(std::remove(this->jsonArray.begin(), this->jsonArray.end(), 
                              ui->brain->text().toStdString()), this->jsonArray.end());
    }
    print_nodes_list();
    // std::cout << "List of nodes to initiate after modifying the array: \n" << this->json_string.dump(4) << std::endl;
}

void MainWindow::print_nodes_list(){
    this->json_string["nodes_to_enable"] = this->jsonArray;
    // QString styleSheet = "color: #607cff; background-color: #242526;";
    // ui->nodes_to_enable->setStyleSheet(styleSheet);
    ui->nodes_to_enable->setReadOnly(false); // Allow modifications
    ui->nodes_to_enable->setPlainText(QString::fromStdString(this->json_string.dump(4)));
    ui->nodes_to_enable->setReadOnly(true); // Restore read-only mode
}

void MainWindow::on_mediator_toggled(bool checked)
{
    if (checked){
        this->jsonArray.push_back(ui->mediator->text().toStdString());
    } else {
        this->jsonArray.erase(std::remove(this->jsonArray.begin(), this->jsonArray.end(), 
                              ui->mediator->text().toStdString()), this->jsonArray.end());
    }
    print_nodes_list();
}

void MainWindow::on_pid_toggled(bool checked)
{
    if (checked){
        this->jsonArray.push_back(ui->pid->text().toStdString());
    } else {
        this->jsonArray.erase(std::remove(this->jsonArray.begin(), this->jsonArray.end(), 
                              ui->pid->text().toStdString()), this->jsonArray.end());
    }
    print_nodes_list();
}
