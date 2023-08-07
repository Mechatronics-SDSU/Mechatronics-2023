#ifndef GUI_HOME_PAGE_HPP
#define GUI_HOME_PAGE_HPP

#include <QWidget>

namespace Ui {
class GUIHomePage;
}

class GUIHomePage : public QWidget
{
    Q_OBJECT

public:
    explicit GUIHomePage(QWidget *parent = nullptr);
    ~GUIHomePage();

private:
    Ui::GUIHomePage *ui;
};

#endif // GUI_HOME_PAGE_HPP
