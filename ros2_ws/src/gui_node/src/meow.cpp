#include "meow.hpp"
#include "ui_meow.h"
#include <mainwindow.hpp>

Meow::Meow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Meow)
{
    ui->setupUi(this);
    node = rclcpp::Node::make_shared("meow_node");
    meow_publisher = node->create_publisher<scion_types::msg::Meow>("meow_data", 10);
}

Meow::~Meow()
{
    delete ui;
}

void Meow::on_meowButton_clicked()
{
   // Meow.
    auto message = scion_types::msg::Meow();
    message.meow_data = "meow <3";
    meow_publisher->publish(message);
}