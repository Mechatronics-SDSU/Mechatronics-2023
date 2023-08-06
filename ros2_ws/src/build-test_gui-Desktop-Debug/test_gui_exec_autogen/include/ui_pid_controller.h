/********************************************************************************
** Form generated from reading UI file 'pid_controller.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PID_CONTROLLER_H
#define UI_PID_CONTROLLER_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PIDController
{
public:
    QLabel *label;
    QPushButton *pushButton;

    void setupUi(QWidget *PIDController)
    {
        if (PIDController->objectName().isEmpty())
            PIDController->setObjectName(QString::fromUtf8("PIDController"));
        PIDController->resize(2281, 1280);
        PIDController->setStyleSheet(QString::fromUtf8(""));
        label = new QLabel(PIDController);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(710, 70, 831, 151));
        label->setStyleSheet(QString::fromUtf8("font: 22pt \"Sans Serif\";"));
        label->setAlignment(Qt::AlignCenter);
        pushButton = new QPushButton(PIDController);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(110, 90, 151, 51));
        pushButton->setStyleSheet(QString::fromUtf8(""));

        retranslateUi(PIDController);

        QMetaObject::connectSlotsByName(PIDController);
    } // setupUi

    void retranslateUi(QWidget *PIDController)
    {
        PIDController->setWindowTitle(QApplication::translate("PIDController", "Form", nullptr));
        label->setText(QApplication::translate("PIDController", "PID CONTROLLER", nullptr));
        pushButton->setText(QApplication::translate("PIDController", "Home", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PIDController: public Ui_PIDController {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PID_CONTROLLER_H
