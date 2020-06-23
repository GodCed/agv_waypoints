#ifndef agv_waypoints_QNODE_HPP_
#define agv_waypoints_QNODE_HPP_


#include <ros/ros.h>
#include <QThread>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <agv_waypoints/data_point.hpp>

namespace agv_waypoints
{
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  class QNode : public QThread
  {
    Q_OBJECT
  public:
    QNode(int argc, char** argv );

    void run();
    void quit();

    void sendGoal(geometry_msgs::PoseStamped pose);
    void cancelGoal();

    ros::Time currentTime();

  Q_SIGNALS:
    void rosShutdown();
    void goalReceived(geometry_msgs::PoseStamped);
    void feedbackReceived(geometry_msgs::PoseStamped);
    void odomReceived(agv_waypoints::DataPoint dataPoint);
    void goalDone(bool);

  private:
    ros::Subscriber goalSubscriber_;
    ros::Subscriber odomSubscriber_;
    std::shared_ptr<MoveBaseClient> actionClient_;
    std::shared_ptr<ros::NodeHandle> nh_;

    tf2_ros::Buffer tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    void doneCallback(const actionlib::SimpleClientGoalState&,
                      const move_base_msgs::MoveBaseResultConstPtr&);
  };

}  // namespace agv_waypoints

#endif /* agv_waypoints_QNODE_HPP_ */
