#include <QFile>
#include <QApplication>

#include "rclcpp/rclcpp.hpp"
#include "mainwindow.h"

class GUI : public rclcpp::Node
{
    public:
        GUI()
        : Node("gui_node")
        {
            
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GUI>();

    std::thread rosThread([&]() {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    });

    QApplication app(argc, argv);
    QFile styleFile("/home/mechatronics/test/ros2_ws/src/gui_node/styles.qss");
    styleFile.open(QFile::ReadOnly);
    QString style(styleFile.readAll());
    app.setStyleSheet(style);
    MainWindow window;

    window.show();
    app.exec();

    rosThread.join();
    rclcpp::shutdown();
    return 0;
}
