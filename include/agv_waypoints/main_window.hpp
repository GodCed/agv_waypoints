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

  private Q_SLOTS:


  protected:
    Ui::MainWindowDesign ui;

    QNode qnode;

  };

}  // namespace agv_waypoints

#endif // agv_waypoints_MAIN_WINDOW_HPP
