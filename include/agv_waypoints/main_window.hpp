#ifndef agv_waypoints_MAIN_WINDOW_HPP
#define agv_waypoints_MAIN_WINDOW_HPP

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

namespace agv_waypoints
{
  class MainWindow : public QMainWindow
  {
    Q_OBJECT

  public:
    MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    virtual ~MainWindow();

  private:
    void sendCurrentWaypoint();
    void loadCurrentRun();
    void saveCurrentRun();
    void setCurrentRunFilename(QString filename);

  private Q_SLOTS:
    void onGoalReceived(geometry_msgs::PoseStamped pose);
    void onFeedbackReceived(geometry_msgs::PoseStamped pose);
    void onGoalDone(bool success);

    void on_buttonBegin_clicked();
    void on_buttonEnd_clicked();
    void on_buttonSend_clicked();
    void on_buttonCancel_clicked();
    void on_buttonSendSelected_clicked();

    void on_actionLoad_triggered();
    void on_actionSave_triggered();
    void on_actionSaveAs_triggered();

  protected:
    Ui::MainWindowDesign ui_;
    QNode qnode_;

    bool recording_ = false;
    int  currentWaypointIndex_ = 0;
    QString currentRunFilename_ = "";
  };

}  // namespace agv_waypoints

#endif // agv_waypoints_MAIN_WINDOW_HPP
