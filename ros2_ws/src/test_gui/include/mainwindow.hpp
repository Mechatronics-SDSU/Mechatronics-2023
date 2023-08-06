#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QMainWindow>
#include <pid_controller.hpp>
#include <mission_planning.hpp>

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


private:
    Ui::MainWindow *ui;
    MissionPlanning _mission_planner;
    PIDController _pid_controller;
};
#endif // MAINWINDOW_H
