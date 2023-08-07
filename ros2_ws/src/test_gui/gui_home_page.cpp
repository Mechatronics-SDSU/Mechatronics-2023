#include "gui_home_page.hpp"
#include "ui_gui_home_page.h"

GUIHomePage::GUIHomePage(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::GUIHomePage)
{
    ui->setupUi(this);
}

GUIHomePage::~GUIHomePage()
{
    delete ui;
}
