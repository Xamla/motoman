#include "motoman_driver/industrial_robot_client/motoman_job_action_server.h"

namespace motoman
{
namespace ros_actions
{
using motoman::motion_ctrl::MotomanMotionCtrl;
using ERROR_CODE = motoman::ros_services::ERROR_CODE;
JobActionServer::JobActionServer(ros::NodeHandle &node_handle, std::string base_name, MotomanMotionCtrl &motoman_com)
    : base_name(base_name), motion_ctrl_(motoman_com), node_(node_handle), has_goal(false)
{
  this->action_server_started = false;
  this->action_server = nullptr;
  ROS_INFO("create JobActionServer.");
}

JobActionServer::Ptr JobActionServer::create(std::string base_name, MotomanMotionCtrl &motoman_com, ros::NodeHandle &node_handle)
{
  return JobActionServer::Ptr(new JobActionServer(node_handle, base_name, motoman_com));
}

void JobActionServer::abort(std::string message_text, ERROR_CODE code)
{
  ROS_INFO(message_text.c_str());
  motoman_msgs::StartJobResult result;
  result.success = false;
  result.message = "Job aborted: " + message_text;
  result.err_no = code;
  this->current_goal_handle.setAborted(result, message_text);
  this->cur_job_state.active = false;
  this->has_goal = false;
}

bool JobActionServer::findTaskNoOfStartedJob(const std::string job_name, int &task_no)
{
  int job_line = -1;
  int step = -1;
  std::string job_name_in_task;
  for (int task_number = 0; task_number < NUMBEROFTASKSLOTS; task_number++)
  {
    bool ret = motion_ctrl_.getCurJob(task_number, job_line, step, job_name_in_task);
    if (!ret)
    {
      ROS_WARN("failed to get details of task: %d", task_number);
    }
    else
    {
      if (strcmp(job_name.c_str(), job_name_in_task.c_str()) == 0)
      {
        task_no = task_number;
        cur_job_state.job_name = job_name_in_task;
        cur_job_state.line_number = job_line;
        cur_job_state.step = step;
        cur_job_state.task_no = task_no;
        return true;
      }
    }
  }
  return false;
}

void JobActionServer::doWork()
{
  if (this->has_goal)
  {
    auto goal = this->current_goal_handle.getGoal();
    int task_no = 0;
    int error_mumber;
    std::string reason_for_abort;
    if (!this->cur_job_state.active)
    {
      if (!this->startJob(goal->job_name))
      {
        return;
      }
      this->cur_job_state.active = true;
    }

    if (!findTaskNoOfStartedJob(goal->job_name, task_no))
    {
      reason_for_abort = "Job was not found in tasks.";
      this->abort(reason_for_abort, ERROR_CODE::spJobNotFound);
      return;
    }
    publishFeedback();
    int error_number;
    int exp_time = 0;
    bool success = motion_ctrl_.waitForJobEnd(this->cur_job_state.task_no, exp_time, error_number);
    if (success)
    {
      motoman_msgs::StartJobResult result;
      switch (error_number)
      {
      case ERROR_CODE::normalEnd:
        result.success = true;
        result.message = "Job finished successfully";
        result.err_no = error_number;
        this->resetMotoRos();
        this->current_goal_handle.setSucceeded(result, "Goal reached");
        this->has_goal = false;
        this->cur_job_state.active = false;
        break;
      case ERROR_CODE::holdPP:
      case ERROR_CODE::holdExternal:
      case ERROR_CODE::holdCommand:
      case ERROR_CODE::errorAlarmStatus:
      case ERROR_CODE::servosOff:
        reason_for_abort = motoman::ros_services::printErrorCode(error_mumber);
        this->abort(reason_for_abort, (ERROR_CODE)error_mumber);
        break;
      }
    }
    else
    {
      reason_for_abort = "could not call waitForJobEnd command. Check connection to robot.";
      this->abort(reason_for_abort, ERROR_CODE::wrongOp);
    }
  }
}

void JobActionServer::start()
{
  this->action_server =
      new ActionServer(this->node_, this->base_name, boost::bind(&JobActionServer::goalCallback, this, _1),
                       boost::bind(&JobActionServer::cancelCallback, this, _1), false);
  this->action_server->start();
  this->action_server_started = true;
}

void JobActionServer::shutdown()
{
  if (this->action_server_started == true)
  {
    delete this->action_server;
    this->action_server = nullptr;
  }
}

void JobActionServer::goalCallback(GoalHandle goal_handle)
{
  auto goal = goal_handle.getGoal();
  ROS_INFO("Received goal request with job name: %s", goal->job_name.c_str());
  std::string reason_for_rejection = "";
  if (goal->job_name.size() > 33)
  {
    reason_for_rejection = "Job_name is too long.";
    motoman_msgs::StartJobResult result;
    result.success = false;
    result.message = reason_for_rejection;
    result.err_no = ERROR_CODE::spJobNotFound;
    goal_handle.setRejected(result, reason_for_rejection);
    return;
  }

  if (this->has_goal)
  {
    reason_for_rejection = "Job is already active.";
    motoman_msgs::StartJobResult result;
    result.success = false;
    result.message = reason_for_rejection;
    result.err_no = ERROR_CODE::wrongOp;
    goal_handle.setRejected(result, reason_for_rejection);
    return;
  }

  goal_handle.setAccepted("Accept job start command");
  ROS_INFO("Accepted goal %s", goal_handle.getGoalID().id.c_str());
  this->current_goal_handle = goal_handle;
  this->cur_job_state.active = false;
  this->cur_job_state.job_name = goal->job_name;
  this->has_goal = true;
}

bool JobActionServer::publishFeedback()
{
  motoman_msgs::StartJobFeedback feedback;
  feedback.job_line = this->cur_job_state.line_number;
  feedback.step = this->cur_job_state.step;
  feedback.message = "Task no: " + std::to_string(this->cur_job_state.task_no);
  this->current_goal_handle.publishFeedback(feedback);
}

void JobActionServer::stopCallback()
{
  this->abort("Received stop callback", ERROR_CODE::normalEnd);
}

void JobActionServer::cancelCallback(GoalHandle goal_handle)
{
  this->abort("Received cancel callback", ERROR_CODE::normalEnd);
}

bool JobActionServer::resetMotoRos()
{
  return startJob("INIT_ROS");
}

bool JobActionServer::startJob(std::string target_job_name)
{
  std::string active_job_name;
  std::string reason_for_abort;
  int task_no = 0;
  int error_mumber;
  int job_line;
  int step;
  if (!motion_ctrl_.getCurJob(0, job_line, step, active_job_name)) // 0 specifies master job
  {
    reason_for_abort = "could not call setHold command. Check connection to robot.";
    this->abort(reason_for_abort, ERROR_CODE::wrongOp);
    return false;
  }

  if (!motion_ctrl_.setHold(1, error_mumber))
  {
    reason_for_abort = "could not call setHold command. Check connection to robot.";
    this->abort(reason_for_abort, ERROR_CODE::wrongOp);
    return false;
  }

  if (!motion_ctrl_.setHold(0, error_mumber))
  {
    reason_for_abort = "could not call setHold command. Check connection to robot.";
    this->abort(reason_for_abort, ERROR_CODE::wrongOp);
    return false;
  }

  if (!motion_ctrl_.startJob(task_no, target_job_name, error_mumber))
  {
    reason_for_abort = "could not call startJob command. Check connection to robot.";
    this->abort(reason_for_abort, ERROR_CODE::wrongOp);
    return false;
  }
  else
  {
    if (error_mumber != ERROR_CODE::normalEnd)
    {
      reason_for_abort = motoman::ros_services::printErrorCode(error_mumber);
      this->abort(reason_for_abort, (ERROR_CODE)error_mumber);
      return false;
    }
  }
  return true;
}

} // namespace ros_actions
} // namespace motoman