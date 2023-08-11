#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QMainWindow>
#include <mission_planning.hpp>
#include <pid_controller.hpp>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include "scion_types/msg/json_string.hpp"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void on_pid_controller_clicked();
    void on_mission_planning_clicked();
//    void moveHome();

    void on_homeButton_clicked();
    // void on_visionButton_clicked();
    // void executeTerminalCommand(const char* command);



    void on_brain_toggled(bool checked);
    void print_nodes_list();

    void on_mediator_toggled(bool checked);

    void on_pid_toggled(bool checked);
    void on_start_nodes_clicked();

private:
    Ui::MainWindow *ui;
    MissionPlanning _mission_planner;
    PIDController _pid_controller;

    using json = nlohmann::json;
    json jsonArray = json::array();
    json json_string;

    rclcpp::Node::SharedPtr json_gui_node;
    rclcpp::Publisher<scion_types::msg::JsonString>::SharedPtr json_string_publisher;
};
#endif // MAINWINDOW_H
