#include <QtGui>
#include <QApplication>
#include <agv_waypoints/main_window.hpp>

Q_DECLARE_METATYPE(geometry_msgs::PoseStamped);

int main(int argc, char **argv)
{
  qRegisterMetaType<geometry_msgs::PoseStamped>();

  QApplication app(argc, argv);

  agv_waypoints::MainWindow w(argc,argv);
  w.show();

  return app.exec();
}
