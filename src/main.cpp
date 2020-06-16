#include <QtGui>
#include <QApplication>
#include <agv_waypoints/main_window.hpp>

int main(int argc, char **argv)
{
  QApplication app(argc, argv);

  agv_waypoints::MainWindow w(argc,argv);
  w.show();

  return app.exec();
}
