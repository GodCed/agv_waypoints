#include <ros/ros.h>
#include <agv_waypoints/qnode.hpp>
#include <agv_waypoints/topics.hpp>
#include <agv_waypoints/data_point.hpp>

namespace agv_waypoints
{
  QNode::QNode(int argc, char** argv )
  {
    ros::init(argc, argv, "run_planner");

    nh_ = std::make_shared<ros::NodeHandle>();
    actionClient_ = std::make_shared<MoveBaseClient>(*nh_, "move_base", true);
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_, *nh_, true);

    goalSubscriber_ = nh_->subscribe<geometry_msgs::PoseStamped>(
          "move_base_simple/goal",
          10,
          &QNode::goalCallback,
          this);

    odomSubscriber_ = nh_->subscribe<nav_msgs::Odometry>(
          "odom",
          10,
          &QNode::odomCallback,
          this);

    ROS_INFO("Waiting for move_base action server.");
    actionClient_->waitForServer();
    ROS_INFO("Connected to move_base action server.");
  }

  void QNode::run()
  {
    ros::spin();
    Q_EMIT rosShutdown();
  }

  void QNode::quit()
  {
    ros::shutdown();
    ros::waitForShutdown();
  }

  void QNode::sendGoal(geometry_msgs::PoseStamped pose)
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = pose;

    actionClient_->sendGoal(
          goal,
          boost::bind(&QNode::doneCallback, this, _1, _2),
          MoveBaseClient::SimpleActiveCallback(),
          boost::bind(&QNode::feedbackCallback, this, _1));
  }

  void QNode::cancelGoal()
  {
    actionClient_->cancelGoal();
  }

  ros::Time QNode::currentTime()
  {
    ros::Time time;
    do {
      time = ros::Time::now();
    } while (!time.isValid() && ros::ok());
    return time;
  }

  void QNode::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    Q_EMIT goalReceived(*msg);
  }

  void QNode::odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    DataPoint dataPoint(*msg);
    try {
      dataPoint.setTransform(tfBuffer_.lookupTransform(
                               "agv1/map",
                               "agv1/base_link",
                               ros::Time(0)));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
    }
    Q_EMIT odomReceived(dataPoint);
  }

  void QNode::feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
  {
    Q_EMIT feedbackReceived(feedback->base_position);
  }

  void QNode::doneCallback(const actionlib::SimpleClientGoalState&,
                    const move_base_msgs::MoveBaseResultConstPtr&)
  {
    Q_EMIT goalDone(actionClient_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  }

}  // namespace agv_waypoints
