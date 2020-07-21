#include <QMessageBox>
#include <QFileDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTime>
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

  QObject::connect(&qnode_, &QNode::odomReceived, this, &MainWindow::onOdomReceived);

  QObject::connect(
        &qnode_, SIGNAL(goalDone(bool)),
        this, SLOT(onGoalDone(bool)));

  QObject::connect(
        &timer_, SIGNAL(timeout()),
        this, SLOT(onTimeout()));

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

void MainWindow::onOdomReceived(agv_waypoints::DataPoint dataPoint)
{
  if (logStarted_) {
    dataPoint.setCurrentWaypoint(currentWaypointIndex_);
    dataPoint.toStream(logStream_);
  }
}

void MainWindow::onGoalDone(bool success)
{
  QString msg;
  if(success) {
    lastWpFailed_ = false;
    msg.sprintf("Reached goal %d", currentWaypointIndex_ +1);
    ui_.statusBar->showMessage(msg);

    if (currentWaypointIndex_ < ui_.listWaypoints->count() -1) {
      currentWaypointIndex_++;
      sendCurrentWaypoint();
    }
    else if (ui_.checkBoxLoop->checkState() == Qt::CheckState::Checked)
    {
      currentWaypointIndex_ = 0;
      sendCurrentWaypoint();
    }
    else {
      on_buttonCancel_clicked();
    }
  }
  else {
    msg.sprintf("Failed to reach goal %d. Retrying...", currentWaypointIndex_ + 1);
    ui_.statusBar->showMessage(msg);
    sendCurrentWaypoint();

    if (!lastWpFailed_) { // If last waypoint failed it's probably just a retry
      ros::Time now = qnode_.currentTime();
      unsigned int elapsed = now.sec - lastFailure_.sec;
      ui_.lineFailureTime->setText(timeStringFromSeconds(elapsed));
      lastFailure_ = now;
    }
    lastWpFailed_ = true;
  }
}

void MainWindow::onTimeout()
{
  unsigned int nowSec = qnode_.currentTime().sec;

  unsigned int elapsed = nowSec - startTime_.sec;
  ui_.lineElapsedTime->setText(timeStringFromSeconds(elapsed));

  elapsed = nowSec - this->lastFailure_.sec;
  ui_.lineElapsedFailure->setText(timeStringFromSeconds(elapsed));
}

void MainWindow::startLog()
{
  if (logStarted_) {
    return;
  }

  QString filename;
  filename.sprintf("agvrun_%lld.csv", QDateTime::currentMSecsSinceEpoch());

  logFile_.setFileName(filename);
  logFile_.open(QIODevice::WriteOnly);

  logStream_.setDevice(&logFile_);
  DataPoint::writeStreamHeaders(logStream_);

  logStarted_ = true;
}

void MainWindow::endLog()
{
  if (!logStarted_) {
    return;
  }

  logFile_.close();
  logStarted_ = false;
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
  lastWpFailed_ = false;
  sendCurrentWaypoint();

  startTime_ = qnode_.currentTime();
  onTimeout();

  lastFailure_ = startTime_;
  ui_.lineFailureTime->setText("");

  timer_.start(1000);
  startLog();
}

void MainWindow::on_buttonCancel_clicked()
{
  qnode_.cancelGoal();

  timer_.stop();
  endLog();
}

void MainWindow::on_buttonSendSelected_clicked()
{
  QList<QListWidgetItem*> selection = ui_.listWaypoints->selectedItems();
  if (selection.length() > 0) {
    currentWaypointIndex_ = ui_.listWaypoints->row(selection.first());
    lastWpFailed_ = false;
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

QString MainWindow::timeStringFromSeconds(unsigned int sec)
{
  if (sec == 0)
  {
    return "";
  }

  unsigned int hours = sec / 3600;
  unsigned int minutes = sec / 60;
  unsigned int seconds = sec % 60;

  QString time;
  time.sprintf("%02d:%02d:%02d", hours, minutes, seconds);

  return time;
}
