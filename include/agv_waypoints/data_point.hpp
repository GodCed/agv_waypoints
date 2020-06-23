#ifndef agv_waypoints_DATA_POINT_HPP_
#define agv_waypoints_DATA_POINT_HPP_


#include <QTextStream>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>


namespace agv_waypoints
{

class DataPoint
{
public:
  DataPoint() {}
  DataPoint(nav_msgs::Odometry odom): agvOdom_{odom} {}

  static void writeStreamHeaders(QTextStream &stream);
  void toStream(QTextStream &stream);

  inline void setTransform(geometry_msgs::TransformStamped transform) {
    agvTransform_ = transform;
  }
  inline void setOdom(nav_msgs::Odometry odom) {
    agvOdom_ = odom;
  }
  inline void setCurrentWaypoint(int index) {
    currentWaypoint_ = index;
  }

private:
  geometry_msgs::TransformStamped agvTransform_;
  nav_msgs::Odometry agvOdom_;
  int currentWaypoint_;
};


}

#endif // agv_waypoints_DATA_POINT_HPP_
