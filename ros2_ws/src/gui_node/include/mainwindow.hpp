#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "scion_types/msg/pid_tuning.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<scion_types::msg::PidTuning>::SharedPtr kp_publisher;
    rclcpp::Publisher<scion_types::msg::PidTuning>::SharedPtr ki_publisher; 
    rclcpp::Publisher<scion_types::msg::PidTuning>::SharedPtr kd_publisher;  
    int axis = 0;  

private slots:
    void handleIndexChanged();
    void updateKpValue();
    void updateKiValue();
    void updateKdValue();
    
};
#endif // MAINWINDOW_H
