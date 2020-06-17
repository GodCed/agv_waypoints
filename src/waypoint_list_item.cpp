#include <agv_waypoints/waypoint_list_item.hpp>

using namespace agv_waypoints;

QVariant WaypointListItem::data(int role) const
{
  int row = this->listWidget()->row(this);
  geometry_msgs::Point pos = pose_.pose.position;

  QString label;
  label.sprintf("%d \t(%.2f,%.2f)", row+1, pos.x, pos.y);

  switch(role) {
    case Qt::DisplayRole: return label;
    default: return QListWidgetItem::data(role);
  }
}
