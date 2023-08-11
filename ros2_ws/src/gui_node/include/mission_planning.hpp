#ifndef MISSION_PLANNING_HPP
#define MISSION_PLANNING_HPP

#include <QWidget>
namespace Ui {
class MissionPlanning;
}

class MissionPlanning : public QWidget
{
    Q_OBJECT

public:
    explicit MissionPlanning(QWidget *parent = nullptr);
    ~MissionPlanning();

private slots:
//    void on_home_button_clicked();

    void on_page2_button_clicked();

//signals:
//    void HomeClicked();

private:
    Ui::MissionPlanning *ui;
};

#endif // MISSION_PLANNING_HPP
