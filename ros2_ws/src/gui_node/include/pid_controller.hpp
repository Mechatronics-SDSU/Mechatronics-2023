#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP
#include "rclcpp/rclcpp.hpp"
#include "scion_types/msg/pid_tuning.hpp"
#include <QWidget>
#include <QTextEdit> 

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
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<scion_types::msg::PidTuning>::SharedPtr kp_publisher;
    rclcpp::Publisher<scion_types::msg::PidTuning>::SharedPtr ki_publisher;
    rclcpp::Publisher<scion_types::msg::PidTuning>::SharedPtr kd_publisher;
    int axis = 0;
    float kpVal = 0.0;
    float kiVal = 0.0;
    float kdVal = 0.0;

protected:
    void keyPressEvent(QKeyEvent *event);

signals:
//    void HomeClicked();

private slots:
//    void on_home_button_clicked();

    void on_KpValue_editingFinished();
    void on_KiValue_editingFinished();
    void on_KdValue_editingFinished();
    void on_Tuning_Axis_currentIndexChanged(int index);
    void executeCommands(QTextEdit* textBoxObject, const scion_types::msg::PidTuning& output);
    void on_kpClearButton_clicked();
    void on_kiClearButton_clicked();
    void on_kdClearButton_clicked();
    void on_clearAllButton_clicked();
    void on_topicListButton_clicked();
    void navigateFocus(Qt::Key key);
};

#endif // PID_CONTROLLER_HPP
