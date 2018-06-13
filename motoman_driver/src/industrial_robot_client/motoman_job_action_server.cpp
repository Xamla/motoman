#include "motoman_driver/industrial_robot_client/motoman_job_action_server.h"

namespace motoman
{
namespace ros_actions
{
using motoman::motion_ctrl::MotomanMotionCtrl;
using ERROR_CODE = motoman::ros_services::ERROR_CODE;
JobActionServer::JobActionServer(ros::NodeHandle &node_handle, std::string base_name, MotomanMotionCtrl &motoman_com)
    : base_name(base_name), motion_ctrl_(motoman_com), node_(node_handle), has_goal(false), job_exe_state(JOB_EXECUTION_STATE::IDLE)
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
  ROS_INFO("%s", message_text.c_str());
  motoman_msgs::StartJobResult result;
  result.success = false;
  result.status_message = "Job aborted: " + message_text;
  result.err_no = code;
  this->current_goal_handle.setAborted(result, message_text);
  this->cur_job_state.active = false;
  this->has_goal = false;
  this->job_exe_state = JOB_EXECUTION_STATE::IDLE;
  this->resetMotoRos();
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
    int error_number;
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
      //this->abort(reason_for_abort, ERROR_CODE::spJobNotFound);
      //this->cur_job_state.message = reason_for_abort;
      return;
    }

    bool success = false;
    int s_hold = -1;
    int s_start = -1;
    success = motion_ctrl_.getPlayStatus(s_start, s_hold, error_number);

    publishFeedback();

    if (s_hold > 0)
    {
      ROS_INFO("Job is Paused");
      this->job_exe_state = JOB_EXECUTION_STATE::ONHOLD;
      this->cur_job_state.is_onhold = true;
      return;
    }
    else if (this->job_exe_state == JOB_EXECUTION_STATE::ONHOLD && s_start > 0)
    {
      ROS_INFO("Job is Restarting");
      this->job_exe_state = JOB_EXECUTION_STATE::RESTARTING;
    }

    int exp_time = 0;
    success = motion_ctrl_.waitForJobEnd(this->cur_job_state.task_no, exp_time, error_number);

    if (success)
    {
      motoman_msgs::StartJobResult result;
      switch (error_number)
      {
      case ERROR_CODE::normalEnd:

        if (this->job_exe_state == JOB_EXECUTION_STATE::RESTARTING)
        {
          ROS_INFO("Restart");
          this->cur_job_state.is_onhold = false;
          break;
        }
        else if (this->job_exe_state == JOB_EXECUTION_STATE::ONHOLD)
        {
          ROS_DEBUG("ON HOLD");
          break;
        }
        else if (this->job_exe_state == JOB_EXECUTION_STATE::IDLE)
        {
          ROS_DEBUG("JOB_EXECUTION_STATE::IDLE");
          break;
        }
        else if (this->job_exe_state == JOB_EXECUTION_STATE::MOVING)
        {
          ROS_DEBUG("JOB_EXECUTION_STATE::MOVING");
        }

        result.success = true;
        result.status_message = "Job finished successfully";
        result.err_no = error_number;
        this->resetMotoRos();
        this->current_goal_handle.setSucceeded(result, "Goal reached");
        this->has_goal = false;
        this->cur_job_state.active = false;
        break;
      case ERROR_CODE::holdPP:
        ROS_INFO("waitForJobEnd: Hold PP active");
        break;
      case ERROR_CODE::holdExternal:
        ROS_INFO("waitForJobEnd: Hold external active");
        break;
      case ERROR_CODE::holdCommand:
        ROS_INFO("waitForJobEnd: Hold command active");
        break;
      case ERROR_CODE::errorAlarmStatus:
        ROS_INFO("waitForJobEnd: Hold command active");
      case ERROR_CODE::servosOff:
        //reason_for_abort = motoman::ros_services::printErrorCode(error_number);
        //this->abort(reason_for_abort, (ERROR_CODE)error_number);
        break;
      case ERROR_CODE::timeout:
        this->job_exe_state = JOB_EXECUTION_STATE::MOVING;
        break;
      default:
        reason_for_abort = motoman::ros_services::printErrorCode(error_number);
        ROS_INFO("%s", reason_for_abort.c_str());
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
  if (this->action_server_started == false)
  {
    this->action_server =
        new ActionServer(this->node_, this->base_name, boost::bind(&JobActionServer::goalCallback, this, _1),
                         boost::bind(&JobActionServer::cancelCallback, this, _1), false);
    this->action_server->start();
    this->action_server_started = true;
  }
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
    result.status_message = reason_for_rejection;
    result.err_no = ERROR_CODE::spJobNotFound;
    goal_handle.setRejected(result, reason_for_rejection);
    return;
  }

  if (this->has_goal)
  {
    reason_for_rejection = "Job is already active.";
    motoman_msgs::StartJobResult result;
    result.success = false;
    result.status_message = reason_for_rejection;
    result.err_no = ERROR_CODE::wrongOp;
    goal_handle.setRejected(result, reason_for_rejection);
    return;
  }

  goal_handle.setAccepted("Accept job start command");
  ROS_INFO("Accepted goal %s", goal_handle.getGoalID().id.c_str());
  this->current_goal_handle = goal_handle;
  this->cur_job_state.active = false;
  this->cur_job_state.job_name = goal->job_name;
  this->cur_job_state.last_onhold = ros::Time::now();
  this->has_goal = true;
}

bool JobActionServer::publishFeedback()
{
  motoman_msgs::StartJobFeedback feedback;
  feedback.job_name = this->cur_job_state.job_name;
  feedback.job_line = this->cur_job_state.line_number;
  feedback.step = this->cur_job_state.step;
  feedback.status_message = "Task no: " + std::to_string(this->cur_job_state.task_no);
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
  ROS_INFO("START JOB");
  std::string active_job_name;
  std::string reason_for_abort;
  int task_no = 0;
  int error_number;
  int job_line;
  int step;
  if (!motion_ctrl_.getCurJob(0, job_line, step, active_job_name)) // 0 specifies master job
  {
    reason_for_abort = "could not call setHold command. Check connection to robot.";
    this->abort(reason_for_abort, ERROR_CODE::wrongOp);
    return false;
  }

  if (!motion_ctrl_.setHold(1, error_number))
  {
    reason_for_abort = "could not call setHold command. Check connection to robot.";
    this->abort(reason_for_abort, ERROR_CODE::wrongOp);
    return false;
  }

  if (!motion_ctrl_.setHold(0, error_number))
  {
    reason_for_abort = "could not call setHold command. Check connection to robot.";
    this->abort(reason_for_abort, ERROR_CODE::wrongOp);
    return false;
  }

  if (!motion_ctrl_.startJob(task_no, target_job_name, error_number))
  {
    reason_for_abort = "could not call startJob command. Check connection to robot.";
    this->abort(reason_for_abort, ERROR_CODE::wrongOp);
    return false;
  }
  else
  {
    if (error_number != ERROR_CODE::normalEnd)
    {
      reason_for_abort = motoman::ros_services::printErrorCode(error_number);
      this->abort(reason_for_abort, (ERROR_CODE)error_number);
      return false;
    }
  }
  this->job_exe_state = JOB_EXECUTION_STATE::MOVING;
  return true;
}

bool JobActionServer::resumeJob()
{
  if (this->job_exe_state == JOB_EXECUTION_STATE::RESTARTING)
  {
    ROS_INFO("RESUME JOB");
    std::string reason_for_abort;
    int error_number;
    if (!motion_ctrl_.setHold(0, error_number))
    {
      reason_for_abort = "could not call setHold command. Check connection to robot.";
      this->abort(reason_for_abort, ERROR_CODE::wrongOp);
      return false;
    }
    else
    {
      if (error_number != ERROR_CODE::normalEnd)
      {
        reason_for_abort = motoman::ros_services::printErrorCode(error_number);
        this->abort(reason_for_abort, (ERROR_CODE)error_number);
        return false;
      }
    }
  }
  ROS_DEBUG("Trigger MOVING");
  this->job_exe_state = JOB_EXECUTION_STATE::IDLE;
  this->cur_job_state.is_onhold = false;
  return true;
}

} // namespace ros_actions
} // namespace motoman