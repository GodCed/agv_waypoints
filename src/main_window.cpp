#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <agv_waypoints/main_window.hpp>

namespace agv_waypoints
{
  MainWindow::MainWindow(int argc, char** argv, QWidget *parent): QMainWindow(parent), qnode(argc,argv)
  {
    ui.setupUi(this);
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    setWindowIcon(QIcon(":/images/icon.png"));

    qnode.start();
  }

  MainWindow::~MainWindow()
  {
    qnode.quit();
  }

}  // namespace agv_waypoints
