#ifndef WAYPOINT_LIST_ITEM_HPP
#define WAYPOINT_LIST_ITEM_HPP

#include <QListWidgetItem>
#include <geometry_msgs/PoseStamped.h>

namespace agv_waypoints
{

class WaypointListItem : public QListWidgetItem
{

public:
  WaypointListItem(geometry_msgs::PoseStamped pose):  pose_(pose) {}
  virtual QVariant data(int role) const;
  inline geometry_msgs::PoseStamped getPose() { return pose_; }

private:
  geometry_msgs::PoseStamped pose_;
};

}



#endif // WAYPOINT_LIST_ITEM_HPP
