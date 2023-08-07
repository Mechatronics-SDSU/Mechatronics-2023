/********************************************************************************
** Form generated from reading UI file 'gui_home_page.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GUI_HOME_PAGE_H
#define UI_GUI_HOME_PAGE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_GUIHomePage
{
public:
    QWidget *widget_3;
    QStackedWidget *stackedWidget;
    QWidget *page;
    QWidget *page_2;
    QWidget *widget;
    QWidget *widget_2;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton;
    QSpacerItem *verticalSpacer_2;
    QPushButton *pushButton_2;
    QSpacerItem *verticalSpacer;
    QPushButton *pushButton_3;
    QSpacerItem *verticalSpacer_3;
    QPushButton *pushButton_4;
    QSpacerItem *verticalSpacer_4;
    QPushButton *pushButton_5;

    void setupUi(QWidget *GUIHomePage)
    {
        if (GUIHomePage->objectName().isEmpty())
            GUIHomePage->setObjectName(QString::fromUtf8("GUIHomePage"));
        GUIHomePage->resize(2281, 1280);
        widget_3 = new QWidget(GUIHomePage);
        widget_3->setObjectName(QString::fromUtf8("widget_3"));
        widget_3->setGeometry(QRect(339, 0, 1941, 1281));
        stackedWidget = new QStackedWidget(widget_3);
        stackedWidget->setObjectName(QString::fromUtf8("stackedWidget"));
        stackedWidget->setGeometry(QRect(49, 49, 1841, 1191));
        page = new QWidget();
        page->setObjectName(QString::fromUtf8("page"));
        stackedWidget->addWidget(page);
        page_2 = new QWidget();
        page_2->setObjectName(QString::fromUtf8("page_2"));
        stackedWidget->addWidget(page_2);
        widget = new QWidget(GUIHomePage);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(0, -1, 341, 1281));
        widget_2 = new QWidget(widget);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        widget_2->setGeometry(QRect(-3, 60, 311, 791));
        verticalLayout = new QVBoxLayout(widget_2);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        pushButton = new QPushButton(widget_2);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setMinimumSize(QSize(0, 0));
        pushButton->setMaximumSize(QSize(255, 50));
        pushButton->setAutoFillBackground(true);

        verticalLayout->addWidget(pushButton);

        verticalSpacer_2 = new QSpacerItem(20, 108, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);

        pushButton_2 = new QPushButton(widget_2);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setMaximumSize(QSize(255, 50));

        verticalLayout->addWidget(pushButton_2);

        verticalSpacer = new QSpacerItem(20, 108, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        pushButton_3 = new QPushButton(widget_2);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));
        pushButton_3->setMaximumSize(QSize(255, 50));

        verticalLayout->addWidget(pushButton_3);

        verticalSpacer_3 = new QSpacerItem(20, 108, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_3);

        pushButton_4 = new QPushButton(widget_2);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));
        pushButton_4->setMaximumSize(QSize(255, 50));

        verticalLayout->addWidget(pushButton_4);

        verticalSpacer_4 = new QSpacerItem(20, 108, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_4);

        pushButton_5 = new QPushButton(widget_2);
        pushButton_5->setObjectName(QString::fromUtf8("pushButton_5"));
        pushButton_5->setMaximumSize(QSize(255, 50));

        verticalLayout->addWidget(pushButton_5);


        retranslateUi(GUIHomePage);

        QMetaObject::connectSlotsByName(GUIHomePage);
    } // setupUi

    void retranslateUi(QWidget *GUIHomePage)
    {
        GUIHomePage->setWindowTitle(QApplication::translate("GUIHomePage", "Form", nullptr));
        pushButton->setText(QApplication::translate("GUIHomePage", "Home", nullptr));
        pushButton_2->setText(QApplication::translate("GUIHomePage", "PID Controller", nullptr));
        pushButton_3->setText(QApplication::translate("GUIHomePage", "Mission Planning", nullptr));
        pushButton_4->setText(QApplication::translate("GUIHomePage", "Other", nullptr));
        pushButton_5->setText(QApplication::translate("GUIHomePage", "Other", nullptr));
    } // retranslateUi

};

namespace Ui {
    class GUIHomePage: public Ui_GUIHomePage {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GUI_HOME_PAGE_H
