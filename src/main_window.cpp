#include <QMessageBox>
#include <QFileDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <fstream>
#include <agv_waypoints/main_window.hpp>
#include <agv_waypoints/waypoint_list_item.hpp>
#include <agv_waypoints/waypoint.hpp>

using namespace agv_waypoints;

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

  on_buttonEnd_clicked();

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

  ui_.buttonBegin->setEnabled(false);
  ui_.buttonEnd->setEnabled(true);
}

void MainWindow::on_buttonEnd_clicked()
{
  recording_ = false;

  ui_.buttonBegin->setEnabled(true);
  ui_.buttonEnd->setEnabled(false);
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
  ui_.listWaypoints->clear();

  QFile file(currentRunFilename_);
  if (!file.open(QIODevice::ReadOnly))
  {
    QMessageBox::warning(
      this,
     "Error",
     "Failed to read file"
    );
    return;
  }

  QJsonDocument run = QJsonDocument::fromJson(file.readAll());
  if (!run.isArray())
  {
    return;
  }
  file.close();

  QJsonArray waypoints = run.array();
  for (auto object: waypoints)
  {
    Waypoint waypoint;
    if (object.isObject())
    {
      waypoint.read(object.toObject());
      ui_.listWaypoints->addItem(new WaypointListItem(waypoint.pose()));
    }
  }
}

void MainWindow::saveCurrentRun()
{
  QFile file(currentRunFilename_);
  if (!file.open(QIODevice::WriteOnly))
  {
    QMessageBox::warning(
      this,
     "Error",
     "Failed to write file"
    );
    return;
  }

  QJsonArray waypoints;
  for (int row = 0; row < ui_.listWaypoints->count(); row++)
  {
    WaypointListItem item = *static_cast<WaypointListItem*>(
      ui_.listWaypoints->item(row));

    QJsonObject waypoint;
    Waypoint(item.getPose()).write(waypoint);
    waypoints.append(waypoint);
  }

  QJsonDocument run(waypoints);
  file.write(run.toJson());
  file.close();
}

void MainWindow::setCurrentRunFilename(QString filename)
{
  currentRunFilename_ = filename;
  ui_.lineName->setText(filename);
}

void MainWindow::on_actionLoad_triggered()
{
  QString file = QFileDialog::getOpenFileName(
        this,
        "Select run file to load.",
        QString(),
        "Run Files (*.agvrun)");

  if (!file.isNull())
  {
    setCurrentRunFilename(file);
    loadCurrentRun();
  }
}

void MainWindow::on_actionSave_triggered()
{
  if (currentRunFilename_ != "")
  {
    saveCurrentRun();
  }
  else
  {
    on_actionSaveAs_triggered();
  }
}

void MainWindow::on_actionSaveAs_triggered()
{
  QString file = QFileDialog::getSaveFileName(
        this,
        "Enter save file name.",
        QString(),
        "Run Files (*.agvrun)");

  if (!file.isNull())
  {
    setCurrentRunFilename(file);
    saveCurrentRun();
  }
}
