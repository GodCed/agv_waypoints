#include <QMessageBox>
#include <fstream>
#include <agv_waypoints/main_window.hpp>
#include <agv_waypoints/waypoint_list_item.hpp>

namespace agv_waypoints
{
  MainWindow::MainWindow(int argc, char** argv, QWidget *parent):
    QMainWindow(parent), qnode_(argc,argv)
  {
    ui_.setupUi(this);

    QObject::connect(
          &qnode_, SIGNAL(rosShutdown()),
          this, SLOT(close()));
    QObject::connect(
          &qnode_, SIGNAL(goalReceived(geometry_msgs::PoseStamped)),
          this, SLOT(onGoalReceived(geometry_msgs::PoseStamped)));
    QObject::connect(
          &qnode_, SIGNAL(feedbackReceived(geometry_msgs::PoseStamped)),
          this, SLOT(onFeedbackReceived(geometry_msgs::PoseStamped)));
    QObject::connect(
          &qnode_, SIGNAL(goalDone(bool)),
          this, SLOT(onGoalDone(bool)));

    setWindowIcon(QIcon(":/images/icon.png"));
    ui_.statusBar->showMessage("Connected to move_base");

    qnode_.start();
  }

  MainWindow::~MainWindow()
  {
    qnode_.quit();
  }

  void MainWindow::onGoalReceived(geometry_msgs::PoseStamped pose)
  {
    if(recording_) {
      ui_.listWaypoints->addItem(new WaypointListItem(pose));
    }
  }

  void MainWindow::onFeedbackReceived(geometry_msgs::PoseStamped)
  {

  }

  void MainWindow::onGoalDone(bool success)
  {
    QString msg;
    if(success) {
      msg.sprintf("Reached goal %d", currentWaypointIndex_ +1);
      ui_.statusBar->showMessage(msg);

      if (currentWaypointIndex_ < ui_.listWaypoints->count() -1) {
        currentWaypointIndex_++;
        sendCurrentWaypoint();
      }
    }
    else {
      msg.sprintf("Failed to reach goal %d", currentWaypointIndex_ + 1);
    }
  }

  void MainWindow::on_buttonBegin_clicked()
  {
    ui_.listWaypoints->clear();
    recording_ = true;
  }

  void MainWindow::on_buttonEnd_clicked()
  {
    recording_ = false;
  }

  void MainWindow::on_buttonSend_clicked()
  {
    currentWaypointIndex_ = 0;
    sendCurrentWaypoint();
  }

  void MainWindow::on_buttonCancel_clicked()
  {
    qnode_.cancelGoal();
  }

  void MainWindow::on_buttonSendSelected_clicked()
  {
    QList<QListWidgetItem*> selection = ui_.listWaypoints->selectedItems();
    if (selection.length() > 0) {
      currentWaypointIndex_ = ui_.listWaypoints->row(selection.first());
      sendCurrentWaypoint();
    }
  }

  void MainWindow::sendCurrentWaypoint()
  {
    WaypointListItem item = *static_cast<WaypointListItem*>(
      ui_.listWaypoints->item(currentWaypointIndex_));
    qnode_.sendGoal(item.getPose());
  }

  void MainWindow::loadCurrentRun()
  {
    std::ifstream file(currentRunFilename_, std::ios::binary);
    geometry_msgs::PoseStamped pose;

    if (file.is_open())
    {
      ui_.listWaypoints->clear();
      ui_.lineName->setText(QString(currentRunFilename_.c_str()));

      while (file.peek() != EOF)
      {
        file.read(reinterpret_cast<char*>(&pose), sizeof(geometry_msgs::PoseStamped));
        ui_.listWaypoints->addItem(new WaypointListItem(pose));
      }
      file.close();
    }
    else
    {
      QMessageBox::warning(this,
                           "Error",
                           "Failed to open file");
    }
  }

  void MainWindow::saveCurrentRun()
  {
    std::ofstream file(currentRunFilename_, std::ios::binary | std::ios::trunc);
    if (file.is_open())
    {
      for(int row = 0; row < ui_.listWaypoints->count(); row++)
      {
        WaypointListItem item = *static_cast<WaypointListItem*>(ui_.listWaypoints->item(row));
        geometry_msgs::PoseStamped pose = item.getPose();
        file.write(reinterpret_cast<char*>(&pose), sizeof (geometry_msgs::PoseStamped));
      }
      file.close();
    }
    else
    {
      QMessageBox::warning(this,
                           "Error",
                           "Failed to open file");
    }
  }

}  // namespace agv_waypoints


