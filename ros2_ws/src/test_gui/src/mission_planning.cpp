#include "mission_planning.hpp"
#include "ui_mission_planning.h"
#include <mainwindow.hpp>

MissionPlanning::MissionPlanning(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MissionPlanning)
{
    ui->setupUi(this);
}

MissionPlanning::~MissionPlanning()
{
    delete ui;
}

void MissionPlanning::on_home_button_clicked()
{
    ui->stackedWidget->setCurrentIndex(0);
}
