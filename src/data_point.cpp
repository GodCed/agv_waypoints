#include <agv_waypoints/data_point.hpp>
#include <tf/tf.h>


using namespace agv_waypoints;


void DataPoint::writeStreamHeaders(QTextStream &stream)
{
  stream << "timestamp, " << "wp, ";
  stream << "px, " << "py, " << "pth, ";
  stream << "vx, " << "vy, " << "vth" << endl;
}


void DataPoint::toStream(QTextStream &stream)
{
  stream << agvOdom_.header.stamp.sec << ", ";
  stream << currentWaypoint_ << ", ";

  stream << agvTransform_.transform.translation.x << ", ";
  stream << agvTransform_.transform.translation.y << ", ";
  stream << tf::getYaw(agvTransform_.transform.rotation) << ", ";

  stream << agvOdom_.twist.twist.linear.x << ", ";
  stream << agvOdom_.twist.twist.linear.y << ", ";
  stream << agvOdom_.twist.twist.angular.z << endl;
}
