#include "pid_controller.hpp"
#include "ui_pid_controller.h"
#include <QComboBox>
#include <QtCore/QString>
#include <iostream>
#include <QTextEdit>
#include <QProcess>
#include <QWidget>
#include <memory> 
#include <QKeyEvent>
#include <QPushButton>
#include <QVBoxLayout>

PIDController::PIDController(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PIDController)
{
    this->node = rclcpp::Node::make_shared("gui_node");
    this->kp_publisher = node->create_publisher<scion_types::msg::PidTuning>("kp_dial_data", 10);
    this->ki_publisher = node->create_publisher<scion_types::msg::PidTuning>("ki_dial_data", 10);
    this->kd_publisher = node->create_publisher<scion_types::msg::PidTuning>("kd_dial_data", 10);
    
    ui->setupUi(this);

    connect(ui->kp_val,  &QLineEdit::returnPressed, this, &PIDController::on_KpValue_editingFinished);
    connect(ui->ki_val,  &QLineEdit::returnPressed, this, &PIDController::on_KiValue_editingFinished);
    connect(ui->kd_val,  &QLineEdit::returnPressed, this, &PIDController::on_KdValue_editingFinished);

    connect(ui->kp_verticalSlider,  SIGNAL(valueChanged(int)), ui->kp_progressBar, SLOT(setValue(int)));
    connect(ui->ki_verticalSlider,  SIGNAL(valueChanged(int)), ui->ki_progressBar, SLOT(setValue(int)));
    connect(ui->kd_verticalSlider,  SIGNAL(valueChanged(int)), ui->kd_progressBar, SLOT(setValue(int)));

}

PIDController::~PIDController()
{
    delete ui;
}

//void PIDController::on_home_button_clicked()
//{
//    emit HomeClicked();
//}

void PIDController::on_KpValue_editingFinished()
{
    this->kpVal = ui->kp_val->text().toFloat();
    auto message = scion_types::msg::PidTuning();
    message.data = this->kpVal;
    message.axis = this->axis;
    kp_publisher->publish(message);
    ui->kp_val->setPlaceholderText(ui->kp_val->text());
    // ui->kp_progressBar->setValue(ui->kp_val->text().toInt() * 100);
    // ui->kp_progressBar->setValue(this->kpPlaceholderVal.toInt() * 100);
    ui->kp_val->clear();
    executeCommands(ui->kpOutput, message);
}

void PIDController::on_KiValue_editingFinished()
{
    this->kiVal = ui->ki_val->text().toFloat();

    auto message = scion_types::msg::PidTuning();
    message.data = this->kiVal;
    message.axis = this->axis;
    ki_publisher->publish(message);
    ui->ki_val->setPlaceholderText(ui->ki_val->text());
    ui->ki_progressBar->setValue(ui->ki_val->text().toInt() * 100);
    ui->ki_val->clear();
    executeCommands(ui->kiOutput, message);
}

void PIDController::on_KdValue_editingFinished()
{
    this->kdVal = ui->kd_val->text().toFloat();

    auto message = scion_types::msg::PidTuning();
    message.data = this->kdVal;
    message.axis = this->axis;
    kd_publisher->publish(message);
    ui->kd_val->setPlaceholderText(ui->kd_val->text());
    ui->kd_progressBar->setValue(ui->kd_val->text().toInt() * 100);
    ui->kd_val->clear();
    executeCommands(ui->kdOutput, message);
}


void PIDController::on_Tuning_Axis_currentIndexChanged(int index)
{
    this->axis = index;
}

 void PIDController::executeCommands(QTextEdit* textBoxObject, const scion_types::msg::PidTuning& output) 
    {
        QString textToDisplay = "Data: " + QString::number(output.data) +
                                ", Axis: " + QString::number(output.axis);

        textBoxObject->setReadOnly(false); // Allow modifications
        textBoxObject->append(textToDisplay);
        textBoxObject->setReadOnly(true); // Restore read-only mode
    }

void PIDController::on_kpClearButton_clicked()
{
    ui->kpOutput->clear();
}

void PIDController::on_kiClearButton_clicked()
{
    ui->kiOutput->clear();
}

void PIDController::on_kdClearButton_clicked()
{
    ui->kdOutput->clear();
}

void PIDController::on_clearAllButton_clicked()
{
    on_kpClearButton_clicked();
    on_kiClearButton_clicked();
    on_kdClearButton_clicked();
}


void PIDController::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Up:
    case Qt::Key_Down:
        navigateFocus(static_cast<Qt::Key>(event->key()));
        break;

    }
}


void PIDController::navigateFocus(Qt::Key key)
{
    if (ui->kp_val->hasFocus()) {
        if (key == Qt::Key_Up) {
            ui->kd_val->setFocus();
        } else if (key == Qt::Key_Down) {
            ui->ki_val->setFocus();
        }
    } else if (ui->ki_val->hasFocus()) {
        if (key == Qt::Key_Up) {
            ui->kp_val->setFocus();
        } else if (key == Qt::Key_Down) {
            ui->kd_val->setFocus();
        }
    } else if (ui->kd_val->hasFocus()) {
        if (key == Qt::Key_Up) {
            ui->ki_val->setFocus();
        } else if (key == Qt::Key_Down) {
            ui->kp_val->setFocus();
        }
    }
}
