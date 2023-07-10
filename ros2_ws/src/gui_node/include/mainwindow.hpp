#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

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
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr kp_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ki_publisher; 
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr kd_publisher;  
    int axis;  

private slots:
    void handleIndexChanged();
    void updateKpValue();
    void updateKiValue();
    void updateKdValue();
    
};
#endif // MAINWINDOW_H
