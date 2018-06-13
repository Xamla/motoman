/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 * Author: Shaun Edwards
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *  * Neither the name of the Southwest Research Institute, nor the names
 *  of its contributors may be used to endorse or promote products derived
 *  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "motoman_driver/industrial_robot_client/motoman_ros_services.h"

namespace motoman
{
namespace ros_services
{
using motoman::motion_ctrl::MotomanMotionCtrl;

MotomanRosServices::MotomanRosServices(MotomanMotionCtrl *connection, ros::NodeHandle *pn)
    : motion_ctrl_(connection), node_(pn)
{
  srv_io_reader_ = node_->advertiseService("io_read", &MotomanRosServices::ioReadCB, this);
  srv_io_writer_ = node_->advertiseService("io_write", &MotomanRosServices::ioWriteCB, this);

  srv_list_jobs_ = node_->advertiseService("list_jobs", &MotomanRosServices::listJobsCB, this);

  srv_hold_ = node_->advertiseService("hold", &MotomanRosServices::holdCB, this);
  srv_startJob_ = node_->advertiseService("start_job", &MotomanRosServices::startJobCB, this);
  srv_waitForJobEnd_ = node_->advertiseService("wait_for_job_end", &MotomanRosServices::waitForJobEndCB, this);

  srv_setMasterJob_ = node_->advertiseService("set_master_job", &MotomanRosServices::setMasterJobCB, this);
  srv_setCurJob_ = node_->advertiseService("set_cur_job", &MotomanRosServices::setCurJobCB, this);
  srv_getCurJob_ = node_->advertiseService("get_cur_job", &MotomanRosServices::getCurJobCB, this);

  srv_getMasterJob_ = node_->advertiseService("get_master_job", &MotomanRosServices::getMasterJobCB, this);
  srv_DeleteJob_ = node_->advertiseService("delete_job", &MotomanRosServices::deleteJobCB, this);

  srv_cancelError_ = node_->advertiseService("cancel_error", &MotomanRosServices::cancelErrorCB, this);
  srv_resetAlarm_ = node_->advertiseService("reset_alarm", &MotomanRosServices::resetAlarmCB, this);
  srv_setAlarm_ = node_->advertiseService("set_alarm", &MotomanRosServices::setAlarmCB, this);
  srv_put_user_vars_ = node_->advertiseService("put_user_vars", &MotomanRosServices::putUserVarsCB, this);
  srv_get_user_vars_ = node_->advertiseService("get_user_vars", &MotomanRosServices::getUserVarsCB, this);

  srv_skillRead_ = node_->advertiseService("skill_read", &MotomanRosServices::skillReadCB, this);
  srv_skillEnd_ = node_->advertiseService("skill_end", &MotomanRosServices::skillEndCB, this);

  srv_getPlayStatus_ = node_->advertiseService("get_play_status", &MotomanRosServices::getPlayStatusCB, this);
  srv_getMode_ = node_->advertiseService("get_mode", &MotomanRosServices::getModeCB, this);

  srv_setServoPower_ = node_->advertiseService("set_servo_power", &MotomanRosServices::setServoPowerCB, this);
  srv_getServoPower_ = node_->advertiseService("get_servo_power", &MotomanRosServices::getServoPowerCB, this);

  srv_getJobDate_ = node_->advertiseService("get_job_date", &MotomanRosServices::getJobDateCB, this);
  srv_status_ = node_->advertiseService("status", &MotomanRosServices::statusCB, this);
  srv_getControllerReady_ = node_->advertiseService("get_controller_ready", &MotomanRosServices::getControllerReadyCB, this);
}

MotomanRosServices::~MotomanRosServices()
{
}

MotomanRosServices::Ptr MotomanRosServices::create(MotomanMotionCtrl *connection, ros::NodeHandle *pn)
{
  return MotomanRosServices::Ptr(new MotomanRosServices(connection, pn));
}

bool MotomanRosServices::statusCB(motoman_msgs::Status::Request &req,
                                  motoman_msgs::Status::Response &res)
{
  res.controller_ready = motion_ctrl_->controllerReady();
  res.play_status.success = motion_ctrl_->getPlayStatus(res.play_status.on_play, res.play_status.on_hold, res.play_status.err_no);
  int powerOn = -1;
  int errorNumber = -1;
  bool success = motion_ctrl_->getServoPower(powerOn, errorNumber);
  res.power_on = powerOn > 0;

  res.operation_mode.success = motion_ctrl_->getMode(res.operation_mode.s_mode, res.operation_mode.s_remote, res.operation_mode.err_no);

  int taskNumber = 0;
  int jobLine = -1;
  int step = -1;
  std::string jobName;
  bool ret = motion_ctrl_->getCurJob(taskNumber, jobLine, step, jobName);
  res.cur_job.success = ret;
  if (!ret)
  {
    res.cur_job.message = "failed to get details of job: " + jobName;
    return true;
  }

  res.cur_job.job_line = jobLine;
  res.cur_job.step = step;
  res.cur_job.job_name = jobName;

  ret = motion_ctrl_->getMasterJob(taskNumber, jobName);
  if (!ret)
  {
    res.cur_job.message = "failed to get master job: " + jobName;
    return true;
  }
  res.cur_job.master_job_name = jobName;

  return true;
}

bool MotomanRosServices::getControllerReadyCB(motoman_msgs::GetControllerReady::Request &req,
                                             motoman_msgs::GetControllerReady::Response &res)
{
  res.ready = motion_ctrl_->controllerReady();
  if (res.ready)
  {
    res.message = "OK";
  }
  else
  {
    res.message = "Uninitialized MotoRos motion";
  }

  return true;
}

bool MotomanRosServices::listJobsCB(motoman_msgs::ListJobs::Request &req,
                                    motoman_msgs::ListJobs::Response &res)
{
  std::vector<std::string> result;
  bool ret = motion_ctrl_->listJobs(result);
  res.success = ret;

  if (!res.success)
  {
    res.message = "Motoman could not query job list.";
    ROS_ERROR_STREAM(res.message);
  }
  else
  {
    res.message = "Motoman successfully received job list";
    ROS_DEBUG_STREAM(res.message);
    res.available_jobs = result;
  }

  return true;
}

bool MotomanRosServices::skillReadCB(motoman_msgs::SkillRead::Request &req,
                                     motoman_msgs::SkillRead::Response &res)
{
  std::vector<int> skillPending;
  std::vector<std::string> cmds;
  res.success = motion_ctrl_->readSkill(skillPending, cmds);
  if (res.success)
  {
    for (size_t i = 0; i < cmds.size(); i++)
    {
      res.cmd.push_back(cmds[i]);
    }

    for (size_t i = 0; i < skillPending.size(); i++)
    {
      res.skill_pending.push_back(skillPending[i]);
    }
  }
  return res.success;
}

bool MotomanRosServices::skillEndCB(motoman_msgs::SkillEnd::Request &req,
                                    motoman_msgs::SkillEnd::Response &res)
{
  res.success = motion_ctrl_->endSkill(req.robot_no);

  return res.success;
}

bool MotomanRosServices::getUserVarsCB(motoman_msgs::GetUserVars::Request &req,
                                       motoman_msgs::GetUserVars::Response &res)
{
  res.success = motion_ctrl_->getUserVars(req, res);
  return true;
}

bool MotomanRosServices::putUserVarsCB(motoman_msgs::PutUserVars::Request &req,
                                       motoman_msgs::PutUserVars::Response &res)
{
  res.success = motion_ctrl_->putUserVars(req, res);
  return true;
}

bool MotomanRosServices::cancelErrorCB(motoman_msgs::CancelError::Request &req,
                                       motoman_msgs::CancelError::Response &res)
{
  int errorNumber;
  bool ret = motion_ctrl_->cancelError(errorNumber);
  res.message = printErrorCode(errorNumber);
  res.success = ret;
  return true;
}

bool MotomanRosServices::deleteJobCB(motoman_msgs::DeleteJob::Request &req,
                                     motoman_msgs::DeleteJob::Response &res)
{
  int errorNumber;
  bool ret = motion_ctrl_->deleteJob(req.job_name, errorNumber);
  res.message = printErrorCode(errorNumber);
  res.success = ERROR_CODE::normalEnd == errorNumber;
  return true;
}

bool MotomanRosServices::getMasterJobCB(motoman_msgs::GetMasterJob::Request &req,
                                        motoman_msgs::GetMasterJob::Response &res)
{
  std::string jobName;
  bool ret = motion_ctrl_->getMasterJob(req.task_no, jobName);
  res.success = ret;
  if (!ret)
  {
    res.message = "failed to get master job: " + jobName;
  }
  res.job_name = jobName;
  return true;
}

bool MotomanRosServices::setAlarmCB(motoman_msgs::SetAlarm::Request &req,
                                    motoman_msgs::SetAlarm::Response &res)
{
  int errorNumber;
  bool ret = motion_ctrl_->setAlarm(req.alm_msg, req.alm_code, req.sub_code, errorNumber);
  res.message = printErrorCode(errorNumber);
  res.err_no = errorNumber;
  res.success = ret;
  return true;
}

bool MotomanRosServices::resetAlarmCB(motoman_msgs::ResetAlarm::Request &req,
                                      motoman_msgs::ResetAlarm::Response &res)
{
  int errorNumber;
  bool ret = motion_ctrl_->resetAlarm(errorNumber);
  res.message = printErrorCode(errorNumber);
  res.success = ret;
  return true;
}

bool MotomanRosServices::setCurJobCB(motoman_msgs::SetCurJob::Request &req,
                                     motoman_msgs::SetCurJob::Response &res)
{
  int errorNumber;
  bool ret = motion_ctrl_->setCurJob(req.job_line, req.job_name, errorNumber);
  res.message = printErrorCode(errorNumber);
  res.success = ret;
  return true;
}

bool MotomanRosServices::getCurJobCB(motoman_msgs::GetCurJob::Request &req,
                                     motoman_msgs::GetCurJob::Response &res)
{
  int taskNumber = req.task_no;
  int jobLine = -1;
  int step = -1;
  std::string jobName;
  bool ret = motion_ctrl_->getCurJob(taskNumber, jobLine, step, jobName);
  res.success = ret;
  if (!ret)
  {
    res.message = "failed to get details of job: " + jobName;
    return true;
  }

  res.job_line = jobLine;
  res.step = step;
  res.job_name = jobName;
  return true;
}

bool MotomanRosServices::setMasterJobCB(motoman_msgs::SetMasterJob::Request &req,
                                        motoman_msgs::SetMasterJob::Response &res)
{

  int errorNumber;
  bool ret = motion_ctrl_->setMasterJob(req.task_no, req.job_name, errorNumber);
  res.success = ret;
  res.message = printErrorCode(errorNumber);
  return true;
}

bool MotomanRosServices::waitForJobEndCB(motoman_msgs::WaitForJobEnd::Request &req,
                                         motoman_msgs::WaitForJobEnd::Response &res)
{
  int errorNumber;
  res.success = motion_ctrl_->waitForJobEnd(req.task_no, req.time, errorNumber);
  res.message = printErrorCode(errorNumber);
  res.err_no = errorNumber; // error code
  return true;
}

bool MotomanRosServices::startJobCB(motoman_msgs::StartJob::Request &req,
                                    motoman_msgs::StartJob::Response &res)
{
  std::string jobName = req.job_name;
  int task_no = req.task_no;
  int errorNumber;
  res.success = motion_ctrl_->startJob(task_no, jobName, errorNumber);
  ROS_INFO("%s", printErrorCode(errorNumber).c_str());
  res.message = printErrorCode(errorNumber);
  res.err_no = errorNumber;
  return true;
}

bool MotomanRosServices::holdCB(motoman_msgs::Hold::Request &req,
                                motoman_msgs::Hold::Response &res)
{

  bool on_hold = req.hold > 0;
  int errorNumber = -1;
  motion_ctrl_->setHold(on_hold, errorNumber);
  res.message = printErrorCode(errorNumber);
  res.success = ERROR_CODE::normalEnd == errorNumber;
  return true;
}

bool MotomanRosServices::getPlayStatusCB(motoman_msgs::GetPlayStatus::Request &req,
                                         motoman_msgs::GetPlayStatus::Response &res)
{
  res.success = motion_ctrl_->getPlayStatus(res.on_play, res.on_hold, res.err_no);

  return res.success;
}

bool MotomanRosServices::getModeCB(motoman_msgs::GetMode::Request &req,
                                   motoman_msgs::GetMode::Response &res)
{
  res.success = motion_ctrl_->getMode(res.s_mode, res.s_remote, res.err_no);

  return res.success;
}

bool MotomanRosServices::setServoPowerCB(motoman_msgs::SetServoPower::Request &req,
                                         motoman_msgs::SetServoPower::Response &res)
{
  int errorNumber = -1;
  res.success = motion_ctrl_->setServoPower(req.power_on ? 1 : 0, errorNumber);
  res.err_no = errorNumber;
  res.message = req.power_on ? "Servo Power ON" : "Servo Power OFF";
  return res.success;
}

bool MotomanRosServices::getServoPowerCB(motoman_msgs::GetServoPower::Request &req,
                                         motoman_msgs::GetServoPower::Response &res)
{
  int powerOn = -1;
  int errorNumber = -1;
  res.success = motion_ctrl_->getServoPower(powerOn, errorNumber);
  res.err_no = errorNumber;
  res.power_on = powerOn > 0;
  res.message = res.power_on ? "Servo Power ON" : "Servo Power OFF";
  return res.success;
}

bool MotomanRosServices::getJobDateCB(motoman_msgs::GetJobDate::Request &req,
                                      motoman_msgs::GetJobDate::Response &res)
{
  int year;
  int month;
  int day;
  int sec;
  int min;
  int hour;
  int elapseTime;
  res.success = motion_ctrl_->getJobDate(req.job_name, year, month, day, hour, min, sec, elapseTime, res.err_no);
  res.elapsed_time = elapseTime;
  res.year = year;
  res.month = month;
  res.day = day;
  res.hour = hour;
  res.minutes = min;
  res.seconds = sec;
  return res.success;
}

/*
 * Valid Addresses (dx100)
 * 10010 - 12567 Universal output #10010 - #12567(2048)
 * 60010 - 60647 Interface panel #60010 - #60647(512)
 * 25010 - 27567 Network input #25010 - #27567(2048)
 * 1000000 - 1000559 Register #1000000 - #1000559(560)
*/

bool MotomanRosServices::ioReadCB(motoman_msgs::ReadIO::Request &req,
                                  motoman_msgs::ReadIO::Response &res)
{
  int result = 0;
  res.success = motion_ctrl_->readFromIO(req.address, &result, res.message);
  res.value = result;

  if (res.success)
  {
    res.message = "OK";
  }

  return true;
}

bool MotomanRosServices::ioWriteCB(motoman_msgs::WriteIO::Request &req,
                                   motoman_msgs::WriteIO::Response &res)
{
  int address = req.address;
  if (address >= 10010 && address <= 12567)
  {
    ROS_INFO_STREAM("Write to Universal output: #" << address);
  }
  else if (address >= 60010 && address <= 60647)
  {
    ROS_INFO_STREAM("Write to Interface panel: #" << address);
  }
  else if (address >= 25010 && address <= 27567)
  {
    ROS_INFO_STREAM("Network input: #" << address);
  }
  else if (address >= 1000000 && address <= 1000559)
  {
    ROS_INFO_STREAM("Register: #" << address);
  }
  else
  {
    res.success = false;
    res.message = "Could not write in this specific address. Out of bounds ";
    ROS_ERROR_STREAM(res.message << address);
    return true;
  }

  bool ret = motion_ctrl_->writeToIO(req.address, req.value, res.message);
  res.success = ret;

  if (res.success)
  {
    res.message = "OK";
  }

  return true;
}

} // namespace ros_services
} // namespace motoman
