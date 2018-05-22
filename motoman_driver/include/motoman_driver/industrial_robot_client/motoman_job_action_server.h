#ifndef ACTION_SERVER_H_
#define ACTION_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <queue>

#include <motoman_msgs/StartJobAction.h>
#include "motoman_driver/motion_ctrl.h"
#include "motoman_driver/industrial_robot_client/motoman_ros_services.h"
namespace motoman
{
namespace ros_actions
{
using motoman::motion_ctrl::MotomanMotionCtrl;
using ERROR_CODE = motoman::ros_services::ERROR_CODE;
typedef actionlib::ActionServer<motoman_msgs::StartJobAction> ActionServer;
typedef ActionServer::GoalHandle GoalHandle;

class JobState
{
public:
  int error_code;
  int line_number;
  int step;
  int task_no;
  bool active;
  std::string job_name;
};

class JobActionServer
{
public:
  typedef boost::shared_ptr<JobActionServer> Ptr;
  static Ptr create(std::string base_name, MotomanMotionCtrl &motoman_com, ros::NodeHandle &node_handle);

  void start();
  void shutdown();
  void doWork();
  inline bool hasActiveGoal()
  {
    return this->has_goal;
  }

private:
  JobActionServer(ros::NodeHandle &node_handle, std::string base_name, MotomanMotionCtrl &motoman_com);
  ros::NodeHandle &node_;
  MotomanMotionCtrl &motion_ctrl_;
  std::string base_name;
  ActionServer *action_server;
  bool action_server_started;
  GoalHandle current_goal_handle;
  bool has_goal;

  void goalCallback(GoalHandle goal_handle);
  void cancelCallback(GoalHandle goal_handle);

  void abort(std::string message_text, ERROR_CODE code);
  void stopCallback();
  bool findTaskNoOfStartedJob(const std::string job_name, int &task_no);
  bool publishFeedback();
  bool startJob(std::string target_job_name);
  bool resetMotoRos();
  const int NUMBEROFTASKSLOTS = 15;
  JobState cur_job_state;
};
} // namespace ros_actions
} // namespace motoman
#endif /* ACTION_SERVER_H_ */
