#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

#include <geometry_msgs/PoseStamped.h>
#include <QJsonObject>

namespace agv_waypoints {

  class Waypoint
  {
  public:
    Waypoint() {}
    Waypoint(geometry_msgs::PoseStamped pose): pose_(pose) {}

    void read(const QJsonObject &json);
    void write(QJsonObject &json) const;

    inline geometry_msgs::PoseStamped pose() { return pose_; }
    void setPose(geometry_msgs::PoseStamped pose) { pose_ = pose; }

  private:
    geometry_msgs::PoseStamped pose_;
  };
}

#endif // WAYPOINT_HPP
