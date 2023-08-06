#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <QWidget>

namespace Ui {
class PIDController;
}

class PIDController : public QWidget
{
    Q_OBJECT

public:
    explicit PIDController(QWidget *parent = nullptr);
    ~PIDController();

private:
    Ui::PIDController *ui;
};

#endif // PID_CONTROLLER_HPP
