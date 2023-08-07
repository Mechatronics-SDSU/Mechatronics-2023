/********************************************************************************
** Form generated from reading UI file 'mission_planning.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MISSION_PLANNING_H
#define UI_MISSION_PLANNING_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MissionPlanning
{
public:
    QStackedWidget *stackedWidget;
    QWidget *page;
    QLabel *label;
    QPushButton *home_button;
    QPushButton *page2_button;
    QWidget *page_2;

    void setupUi(QWidget *MissionPlanning)
    {
        if (MissionPlanning->objectName().isEmpty())
            MissionPlanning->setObjectName(QString::fromUtf8("MissionPlanning"));
        MissionPlanning->resize(2281, 1280);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MissionPlanning->sizePolicy().hasHeightForWidth());
        MissionPlanning->setSizePolicy(sizePolicy);
        MissionPlanning->setStyleSheet(QString::fromUtf8(""));
        stackedWidget = new QStackedWidget(MissionPlanning);
        stackedWidget->setObjectName(QString::fromUtf8("stackedWidget"));
        stackedWidget->setGeometry(QRect(0, 0, 2281, 1280));
        stackedWidget->setStyleSheet(QString::fromUtf8(""));
        page = new QWidget();
        page->setObjectName(QString::fromUtf8("page"));
        page->setStyleSheet(QString::fromUtf8(""));
        label = new QLabel(page);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(720, 70, 821, 161));
        label->setStyleSheet(QString::fromUtf8("font: 22pt \"Sans Serif\";"));
        label->setAlignment(Qt::AlignCenter);
        home_button = new QPushButton(page);
        home_button->setObjectName(QString::fromUtf8("home_button"));
        home_button->setGeometry(QRect(80, 60, 161, 61));
        home_button->setStyleSheet(QString::fromUtf8(""));
        page2_button = new QPushButton(page);
        page2_button->setObjectName(QString::fromUtf8("page2_button"));
        page2_button->setGeometry(QRect(1950, 50, 181, 61));
        page2_button->setStyleSheet(QString::fromUtf8(""));
        stackedWidget->addWidget(page);
        page_2 = new QWidget();
        page_2->setObjectName(QString::fromUtf8("page_2"));
        stackedWidget->addWidget(page_2);

        retranslateUi(MissionPlanning);

        stackedWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MissionPlanning);
    } // setupUi

    void retranslateUi(QWidget *MissionPlanning)
    {
        MissionPlanning->setWindowTitle(QApplication::translate("MissionPlanning", "Form", nullptr));
        label->setText(QApplication::translate("MissionPlanning", "MISSION PLANNING", nullptr));
        home_button->setText(QApplication::translate("MissionPlanning", "Home", nullptr));
        page2_button->setText(QApplication::translate("MissionPlanning", "Page 2", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MissionPlanning: public Ui_MissionPlanning {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MISSION_PLANNING_H
