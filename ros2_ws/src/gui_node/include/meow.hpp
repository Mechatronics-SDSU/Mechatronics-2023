#ifndef MEOW_HPP
#define MEOW_HPP

#include <QWidget>
#include "rclcpp/rclcpp.hpp"
#include "scion_types/msg/meow.hpp"
#include <string>

namespace Ui {
class Meow;
}

class Meow : public QWidget
{
    Q_OBJECT

public:
    explicit Meow(QWidget *parent = nullptr);
    ~Meow();

private slots:
    void on_meowButton_clicked();

private:
    Ui::Meow *ui;
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<scion_types::msg::Meow>::SharedPtr meow_publisher;
};

#endif // MEOW_PLANNING_HPP
