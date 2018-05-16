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

enum ERROR_CODE
{
  normalEnd = 0x0000,
  robotInOperation = 0x2010,
  holdPP = 0x2030,
  holdExternal = 0x2040,
  holdCommand = 0x2050,
  errorAlarmStatus = 0x2060,
  servosOff = 0x2070,
  wrongOp = 0x2080,
  inaccessibleData = 0x2110,
  noOpMasterJob = 0x3400,
  jobNameExists = 0x3410,
  editLockJob = 0x4020,
  spJobNotFound = 0x4040,
  overDataRange = 0x5200,
  timeout = 0xFFFF
};

std::string printErrorCode(int error_no)
{
  std::string result;
  switch (error_no)
  {
  case ERROR_CODE::normalEnd:
    result = "Normal end";
    std::cout << std::hex << error_no << " error number: Normal end" << std::endl;
    break;
  case ERROR_CODE::noOpMasterJob:
    result = "Cannot operate MASTER JOB.";
    std::cout << std::hex << error_no << " error number: Cannot operate MASTER JOB." << std::endl;
    break;
  case ERROR_CODE::jobNameExists:
    result = "The JOB name is already registered in another task.";
    std::cout << std::hex << error_no << " error number: The JOB name is already registered in another task." << std::endl;
    break;
  case ERROR_CODE::editLockJob:
    result = "Edit lock job";
    std::cout << std::hex << error_no << " error number: Edit lock job" << std::endl;
    break;
  case ERROR_CODE::robotInOperation:
    result = "Robot is in operation";
    std::cout << std::hex << error_no << " error number: Robot is in operation" << std::endl;
    break;
  case ERROR_CODE::overDataRange:
    result = "Over data range";
    std::cout << std::hex << error_no << " error number: Over data range" << std::endl;
    break;
  case ERROR_CODE::holdPP:
    result = "In HOLD status (PP)";
    std::cout << std::hex << error_no << " error number: In HOLD status (PP)" << std::endl;
    break;
  case ERROR_CODE::holdExternal:
    result = "In HOLD status (External)";
    std::cout << std::hex << error_no << " error number: In HOLD status (External)" << std::endl;
    break;
  case ERROR_CODE::holdCommand:
    result = "In HOLD status (Command)";
    std::cout << std::hex << error_no << " error number: In HOLD status (Command)" << std::endl;
    break;
  case ERROR_CODE::errorAlarmStatus:
    std::cout << std::hex << error_no << " error number: In error/alarm status" << std::endl;
    result = "In error/alarm status";
    break;
  case ERROR_CODE::servosOff:
    std::cout << std::hex << error_no << " error number: In SERVO OFF status" << std::endl;
    result = "In SERVO OFF status";
    break;
  case ERROR_CODE::wrongOp:
    std::cout << std::hex << error_no << " error number: Wrong operation mode" << std::endl;
    result = "Wrong operation mode";
    break;
  case ERROR_CODE::timeout:
    std::cout << std::hex << error_no << " error number: Timeout" << std::endl;
    result = "Timeout";
    break;
  case ERROR_CODE::inaccessibleData:
    std::cout << std::hex << error_no << " error number: Inaccessible Data" << std::endl;
    result = "Inaccessible Data";
    break;
  case ERROR_CODE::spJobNotFound:
    std::cout << std::hex << error_no << " error number: Specified JOB not found" << std::endl;
    result = "Specified JOB not found";
    break;
  default:
    std::cout << std::hex << error_no << " error number" << std::endl;
    result = "unknown error";
    break;
  }
  return result;
}

MotomanRosServices::MotomanRosServices(MotomanMotionCtrl *connection, ros::NodeHandle *pn)
    : motion_ctrl_(connection), node_(pn)
{
  srv_io_reader_ = node_->advertiseService("io_read", &MotomanRosServices::ioReadCB, this);
  srv_io_writer_ = node_->advertiseService("io_write", &MotomanRosServices::ioWriteCB, this);

  srv_list_jobs_ = node_->advertiseService("list_jobs", &MotomanRosServices::listJobsCB, this);

  srv_hold_ = node_->advertiseService("hold_job", &MotomanRosServices::holdCB, this);
  srv_startJob_ = node_->advertiseService("start_job", &MotomanRosServices::startJobCB, this);
  srv_waitForJobEnd_ = node_->advertiseService("wait_for_job_end", &MotomanRosServices::waitForJobEndCB, this);

  srv_setMasterJob_ = node_->advertiseService("set_master_job", &MotomanRosServices::setMasterJobCB, this);
  srv_setCurJob_ = node_->advertiseService("set_cur_job", &MotomanRosServices::setCurJobCB, this);
  srv_getCurJob_ = node_->advertiseService("get_cur_job", &MotomanRosServices::getCurJobCB, this);

  srv_getMasterJob_ = node_->advertiseService("get_master_job", &MotomanRosServices::getMasterJobCB, this);
  srv_DeleteJob_ = node_->advertiseService("delete_job", &MotomanRosServices::deleteJobCB, this);

  srv_cancelError_ = node_->advertiseService("cancel_error", &MotomanRosServices::cancelErrorCB, this);
  srv_resetAlarm_ = node_->advertiseService("reset_alarm", &MotomanRosServices::resetAlarmCB, this);
  srv_put_user_vars_ = node_->advertiseService("put_user_vars", &MotomanRosServices::putUserVarsCB, this);
  srv_get_user_vars_ = node_->advertiseService("get_user_vars", &MotomanRosServices::getUserVarsCB, this);
}

MotomanRosServices::~MotomanRosServices()
{
}

MotomanRosServices::Ptr MotomanRosServices::create(MotomanMotionCtrl *connection, ros::NodeHandle *pn)
{
  return MotomanRosServices::Ptr(new MotomanRosServices(connection, pn));
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

bool MotomanRosServices::getUserVarsCB(motoman_msgs::GetUserVars::Request &req,
                                       motoman_msgs::GetUserVars::Response &res)
{
  bool ret = motion_ctrl_->getUserVars(req,res);
  return true;
}

bool MotomanRosServices::putUserVarsCB(motoman_msgs::PutUserVars::Request &req,
                                       motoman_msgs::PutUserVars::Response &res)
{
  bool ret = motion_ctrl_->putUserVars(req,res);
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
  res.success = ret;
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
  res.success = true;
  return true;
}

/*
 * Valid Adresses (dx100)
 * 10010 - 12567 Universal output #10010 - #12567(2048)
 * 60010 - 60647 Interface panel #60010 - #60647(512)
 * 25010 - 27567 Network input #25010 - #27567(2048)
 * 1000000 - 1000559 Register #1000000 - #1000559(560)
*/

bool MotomanRosServices::ioReadCB(motoman_msgs::ReadIO::Request &req,
                                  motoman_msgs::ReadIO::Response &res)
{
  int result = 0;
  bool ret = motion_ctrl_->readFromIO(req.adress, &result);
  res.success = ret;
  res.value = result;

  if (!res.success)
  {
    res.message = "Could not read from adress";
    ROS_ERROR_STREAM(res.message);
  }
  else
  {
    res.message = "OK";
    ROS_WARN_STREAM(res.message);
  }

  return true;
}

bool MotomanRosServices::ioWriteCB(motoman_msgs::WriteIO::Request &req,
                                   motoman_msgs::WriteIO::Response &res)
{
  int adress = req.adress;
  if (adress >= 10010 && adress <= 12567)
  {
    ROS_INFO_STREAM("Write to Universal output: #" << adress);
  }
  else if (adress >= 60010 && adress <= 60647)
  {
    ROS_INFO_STREAM("Write to Interface panel: #" << adress);
  }
  else if (adress >= 25010 && adress <= 27567)
  {
    ROS_INFO_STREAM("Network input: #" << adress);
  }
  else if (adress >= 1000000 && adress <= 1000559)
  {
    ROS_INFO_STREAM("Register: #" << adress);
  }
  else
  {
    res.success = false;
    res.message = "Could not write in this specific adress. Out of bounds ";
    ROS_ERROR_STREAM(res.message << adress);
    return true;
  }

  bool ret = motion_ctrl_->writeToIO(req.adress, req.value);
  res.success = ret;

  if (!res.success)
  {
    res.message = "Could not read from adress";
    ROS_ERROR_STREAM(res.message);
  }
  else
  {
    res.message = "OK";
  }

  return true;
}

} // namespace ros_services
} // namespace motoman
