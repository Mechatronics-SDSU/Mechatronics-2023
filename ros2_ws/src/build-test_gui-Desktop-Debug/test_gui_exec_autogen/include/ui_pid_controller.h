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
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PIDController
{
public:
    QLabel *label;
    QPushButton *home_button;
    QGroupBox *groupBox;
    QHBoxLayout *horizontalLayout_4;
    QWidget *widget_2;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label_2;
    QLineEdit *lineEdit;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_4;
    QLineEdit *lineEdit_3;
    QSpacerItem *verticalSpacer_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_3;
    QLineEdit *lineEdit_2;
    QGroupBox *groupBox_2;
    QHBoxLayout *horizontalLayout_5;
    QSlider *verticalSlider_2;
    QSlider *verticalSlider_4;
    QSlider *verticalSlider_3;

    void setupUi(QWidget *PIDController)
    {
        if (PIDController->objectName().isEmpty())
            PIDController->setObjectName(QString::fromUtf8("PIDController"));
        PIDController->resize(2288, 1391);
        PIDController->setStyleSheet(QString::fromUtf8(""));
        label = new QLabel(PIDController);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(710, 70, 831, 151));
        label->setStyleSheet(QString::fromUtf8("font: 22pt \"Sans Serif\";"));
        label->setAlignment(Qt::AlignCenter);
        home_button = new QPushButton(PIDController);
        home_button->setObjectName(QString::fromUtf8("home_button"));
        home_button->setGeometry(QRect(110, 90, 151, 51));
        home_button->setStyleSheet(QString::fromUtf8(""));
        groupBox = new QGroupBox(PIDController);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(1222, 470, 541, 681));
        horizontalLayout_4 = new QHBoxLayout(groupBox);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        widget_2 = new QWidget(groupBox);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        widget_2->setMaximumSize(QSize(225, 455));
        verticalLayout = new QVBoxLayout(widget_2);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_2 = new QLabel(widget_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout->addWidget(label_2);

        lineEdit = new QLineEdit(widget_2);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));

        horizontalLayout->addWidget(lineEdit);


        verticalLayout->addLayout(horizontalLayout);

        verticalSpacer = new QSpacerItem(20, 124, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_4 = new QLabel(widget_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_3->addWidget(label_4);

        lineEdit_3 = new QLineEdit(widget_2);
        lineEdit_3->setObjectName(QString::fromUtf8("lineEdit_3"));

        horizontalLayout_3->addWidget(lineEdit_3);


        verticalLayout->addLayout(horizontalLayout_3);

        verticalSpacer_2 = new QSpacerItem(20, 124, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_3 = new QLabel(widget_2);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_2->addWidget(label_3);

        lineEdit_2 = new QLineEdit(widget_2);
        lineEdit_2->setObjectName(QString::fromUtf8("lineEdit_2"));

        horizontalLayout_2->addWidget(lineEdit_2);


        verticalLayout->addLayout(horizontalLayout_2);


        horizontalLayout_4->addWidget(widget_2);

        groupBox_2 = new QGroupBox(PIDController);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(560, 390, 551, 581));
        groupBox_2->setStyleSheet(QString::fromUtf8(""));
        horizontalLayout_5 = new QHBoxLayout(groupBox_2);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        verticalSlider_2 = new QSlider(groupBox_2);
        verticalSlider_2->setObjectName(QString::fromUtf8("verticalSlider_2"));
        verticalSlider_2->setOrientation(Qt::Vertical);

        horizontalLayout_5->addWidget(verticalSlider_2);

        verticalSlider_4 = new QSlider(groupBox_2);
        verticalSlider_4->setObjectName(QString::fromUtf8("verticalSlider_4"));
        verticalSlider_4->setOrientation(Qt::Vertical);

        horizontalLayout_5->addWidget(verticalSlider_4);

        verticalSlider_3 = new QSlider(groupBox_2);
        verticalSlider_3->setObjectName(QString::fromUtf8("verticalSlider_3"));
        verticalSlider_3->setOrientation(Qt::Vertical);

        horizontalLayout_5->addWidget(verticalSlider_3);


        retranslateUi(PIDController);

        QMetaObject::connectSlotsByName(PIDController);
    } // setupUi

    void retranslateUi(QWidget *PIDController)
    {
        PIDController->setWindowTitle(QApplication::translate("PIDController", "Form", nullptr));
        label->setText(QApplication::translate("PIDController", "PID CONTROLLER", nullptr));
        home_button->setText(QApplication::translate("PIDController", "Home", nullptr));
        groupBox->setTitle(QString());
        label_2->setText(QApplication::translate("PIDController", "Kp", nullptr));
        lineEdit->setPlaceholderText(QApplication::translate("PIDController", "Enter kp value", nullptr));
        label_4->setText(QApplication::translate("PIDController", "Ki", nullptr));
        lineEdit_3->setPlaceholderText(QApplication::translate("PIDController", "Enter Ki value", nullptr));
        label_3->setText(QApplication::translate("PIDController", "Kd", nullptr));
        lineEdit_2->setPlaceholderText(QApplication::translate("PIDController", "Enter Kd value", nullptr));
        groupBox_2->setTitle(QString());
    } // retranslateUi

};

namespace Ui {
    class PIDController: public Ui_PIDController {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PID_CONTROLLER_H
