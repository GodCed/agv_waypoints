#include <agv_waypoints/waypoint.hpp>

using namespace agv_waypoints;

void Waypoint::read(const QJsonObject &json)
{
  pose_ = geometry_msgs::PoseStamped();

  if (json.contains("frame_id") && json["frame_id"].isString())
  {
    pose_.header.frame_id = json["frame_id"].toString().toStdString();
  }

  if (json.contains("position") && json["position"].isObject())
  {
    QJsonObject pos = json["position"].toObject();
    if (pos.contains("x") && pos["x"].isDouble())
    {
      pose_.pose.position.x = pos["x"].toDouble();
    }
    if (pos.contains("y") && pos["y"].isDouble())
    {
      pose_.pose.position.y = pos["y"].toDouble();
    }
    if (pos.contains("z") && pos["z"].isDouble())
    {
      pose_.pose.position.z = pos["z"].toDouble();
    }
  }

  if (json.contains("orientation") && json["orientation"].isObject())
  {
    QJsonObject quat = json["orientation"].toObject();
    if (quat.contains("w") && quat["w"].isDouble())
    {
      pose_.pose.orientation.w = quat["w"].toDouble();
    }
    if (quat.contains("x") && quat["x"].isDouble())
    {
      pose_.pose.orientation.x = quat["x"].toDouble();
    }
    if (quat.contains("y") && quat["y"].isDouble())
    {
      pose_.pose.orientation.y = quat["y"].toDouble();
    }
    if (quat.contains("z") && quat["z"].isDouble())
    {
      pose_.pose.orientation.z = quat["z"].toDouble();
    }
  }
}

void Waypoint::write(QJsonObject &json) const
{
  json["frame_id"] = QString(pose_.header.frame_id.c_str());

  QJsonObject pos;
  pos["x"] = pose_.pose.position.x;
  pos["y"] = pose_.pose.position.y;
  pos["z"] = pose_.pose.position.z;
  json["position"] = pos;

  QJsonObject quat;
  quat["w"] = pose_.pose.orientation.w;
  quat["x"] = pose_.pose.orientation.x;
  quat["y"] = pose_.pose.orientation.y;
  quat["z"] = pose_.pose.orientation.z;
  json["orientation"] = quat;
}
