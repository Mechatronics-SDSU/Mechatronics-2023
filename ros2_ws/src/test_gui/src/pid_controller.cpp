#include "pid_controller.hpp"
#include "ui_pid_controller.h"

PIDController::PIDController(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PIDController)
{
    ui->setupUi(this);
}

PIDController::~PIDController()
{
    delete ui;
}
