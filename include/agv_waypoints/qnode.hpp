#ifndef agv_waypoints_QNODE_HPP_
#define agv_waypoints_QNODE_HPP_


#include <ros/ros.h>
#include <QThread>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

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

  Q_SIGNALS:
    void rosShutdown();
    void goalReceived(geometry_msgs::PoseStamped);
    void feedbackReceived(geometry_msgs::PoseStamped);
    void goalDone(bool);

  private:
    ros::Subscriber goalSubscriber_;
    std::shared_ptr<MoveBaseClient> actionClient_;
    std::shared_ptr<ros::NodeHandle> nh_;

    void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    void doneCallback(const actionlib::SimpleClientGoalState&,
                      const move_base_msgs::MoveBaseResultConstPtr&);
  };

}  // namespace agv_waypoints

#endif /* agv_waypoints_QNODE_HPP_ */
