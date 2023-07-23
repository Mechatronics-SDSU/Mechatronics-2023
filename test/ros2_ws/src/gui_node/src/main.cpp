#include <QFile>
#include <QApplication>
#include <chrono>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "mainwindow.hpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    QFile styleFile("/home/mechatronics/master/ros2_ws/src/gui_node/styles.qss");
    styleFile.open(QFile::ReadOnly);
    QString style(styleFile.readAll());
    app.setStyleSheet(style);
    MainWindow window;
    window.show();
    app.exec();
    rclcpp::shutdown();
    return 0;
}