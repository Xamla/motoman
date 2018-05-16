/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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

#include "motoman_driver/motion_ctrl.h"
#include "motoman_driver/simple_message/motoman_motion_ctrl_message.h"
#include "motoman_driver/simple_message/motoman_motion_reply_message.h"

#include "motoman_driver/simple_message/motoman_read_single_io.h"
#include "motoman_driver/simple_message/motoman_read_single_io_reply.h"
#include "motoman_driver/simple_message/messages/motoman_read_single_io_message.h"
#include "motoman_driver/simple_message/messages/motoman_read_single_io_reply_message.h"

#include "motoman_driver/simple_message/motoman_write_single_io.h"
#include "motoman_driver/simple_message/motoman_write_single_io_reply.h"
#include "motoman_driver/simple_message/messages/motoman_write_single_io_message.h"
#include "motoman_driver/simple_message/messages/motoman_write_single_io_reply_message.h"

#include "motoman_driver/simple_message/motoman_simple_rpc.h"
#include "motoman_driver/simple_message/motoman_simple_rpc_reply.h"
#include "motoman_driver/simple_message/messages/motoman_simple_rpc_message.h"
#include "motoman_driver/simple_message/messages/motoman_simple_rpc_reply_message.h"

#include "ros/ros.h"
#include "simple_message/simple_message.h"

#include <string>

namespace RpcCmds = motoman::simple_message::rpc_ctrl::RpcCmds;
namespace MotionControlCmds = motoman::simple_message::motion_ctrl::MotionControlCmds;
namespace MotionReplyResults = motoman::simple_message::motion_reply::MotionReplyResults;
namespace ReadSingleIOReplyResults = motoman::simple_message::io_ctrl_reply::ReadSingleIOReplyResults;
namespace WriteSingleIOReplyResults = motoman::simple_message::io_ctrl_reply::WriteSingleIOReplyResults;
using motoman::simple_message::motion_ctrl::MotionCtrl;
using motoman::simple_message::motion_ctrl_message::MotionCtrlMessage;
using motoman::simple_message::motion_reply_message::MotionReplyMessage;

using motoman::simple_message::io_ctrl::ReadSingleIO;
using motoman::simple_message::io_ctrl_message::ReadSingleIOMessage;
using motoman::simple_message::io_ctrl_reply::ReadSingleIOReply;
using motoman::simple_message::io_ctrl_reply_message::ReadSingleIOReplyMessage;

using motoman::simple_message::io_ctrl::WriteSingleIO;
using motoman::simple_message::io_ctrl_message::WriteSingleIOMessage;
using motoman::simple_message::io_ctrl_reply::WriteSingleIOReply;
using motoman::simple_message::io_ctrl_reply_message::WriteSingleIOReplyMessage;

using motoman::simple_message::rpc_ctrl::SimpleRpc;
using motoman::simple_message::rpc_ctrl_message::SimpleRpcMessage;
using motoman::simple_message::rpc_ctrl_reply::SimpleRpcReply;
using motoman::simple_message::rpc_ctrl_reply_message::SimpleRpcReplyMessage;

using industrial::simple_message::SimpleMessage;

namespace motoman
{
namespace motion_ctrl
{

boost::mutex MotomanMotionCtrl::mutex_;

static inline uint16_t Swap16(uint16_t value)
{
#ifdef BYTE_SWAPPING
  return ((value >> 8) & 0xff) | ((value << 8) & 0xff00);
#else
  return value;
#endif
}

static inline int16_t Swap16(int16_t value)
{
  return (int16_t)Swap16((uint16_t)value);
}

static inline uint32_t Swap32(uint32_t value)
{
#ifdef BYTE_SWAPPING
  return ((uint32_t)Swap16((uint16_t)value) << 16) | (uint32_t)Swap16((uint16_t)(value >> 16));
#else
  return value;
#endif
}

static inline int32_t Swap32(int32_t value)
{
  return (int32_t)Swap32((uint32_t)value);
}

static inline long Swap32(long value)
{
  uint32_t tmp = Swap32(*(uint32_t *)&value);
  return *(long *)&tmp;
}

static inline float Swap32(float value)
{
  uint32_t tmp = Swap32(*(uint32_t *)&value);
  return *(float *)&tmp;
}

void bswap(MP_TASK_SEND_DATA &value)
{
  value.sTaskNo = Swap16(value.sTaskNo);
}

void bswap(MP_CUR_JOB_RSP_DATA &value)
{
  value.usJobLine = Swap16(value.usJobLine);
  value.usStep = Swap16(value.usStep);
}

void bswap(MP_START_JOB_SEND_DATA &value)
{
  value.sTaskNo = Swap16(value.sTaskNo);
}

void bswap(MP_HOLD_SEND_DATA &value)
{
  value.sHold = Swap16(value.sHold);
}

void bswap(MP_STD_RSP_DATA &value)
{
  value.err_no = Swap16(value.err_no);
}

void bswap(MP_WAIT_JOB_SEND_DATA &value)
{
  value.sTaskNo = Swap16(value.sTaskNo);
  value.sTime = Swap16(value.sTime);
}

void bswap(MP_CUR_JOB_SEND_DATA &value)
{
  value.usJobLine = Swap16(value.usJobLine);
}

void bswap(MP_JOB_POS_DATA &value)
{
  value.ctrl_grp = Swap32(value.ctrl_grp);
  value.posType = Swap32(value.posType);
  value.varIndex = Swap32(value.varIndex);
  value.attr = Swap32(value.attr);
  value.attrExt = Swap32(value.attrExt);

  for (size_t i = 0; i < 8; i++)
  {
    value.pos[i] = Swap32(value.pos[i]);
  }
}

void bswap(MP_MOV_CTRL_DATA &value)
{
  value.speedValue = Swap32(value.speedValue);
  value.posNum = Swap32(value.posNum);

  for (size_t i = 0; i < 3; i++)
  {
    bswap(value.posData[i]);
  }
}

void bswap(MP_JOB_STEP_RSP_DATA &value)
{
  value.err_no = Swap16(value.err_no);
  value.attr = Swap32(value.attr);
  value.movCtrlDataNum = Swap32(value.movCtrlDataNum);

  for (size_t i = 0; i < 4; i++)
  {
    bswap(value.movCtrlData[i]);
  }

  value.posLevel = Swap32(value.posLevel);
  value.cornerRadius = Swap32(value.cornerRadius);
  value.accValue = Swap32(value.accValue);
  value.decValue = Swap32(value.decValue);
}

void bswap(MP_MASTER_JOB_SEND_DATA &value)
{
  value.sTaskNo = Swap16(value.sTaskNo);
}

void bswap(MP_JOB_STEP_NO_SEND_DATA &value)
{
  value.usStep = Swap16(value.usStep);
}

void bswap(MP_USR_VAR_INFO &value)
{
  value.var_type = Swap32(value.var_type);
  value.var_no = Swap32(value.var_no);
  switch (value.var_type)
  {
  case 1:
    value.val.i = Swap16(value.val.i);
    break;
  case 2:
    value.val.d = Swap32(value.val.d);
    break;
  case 3:
    value.val.r = Swap32(value.val.r);
    break;
  }
}

bool MotomanMotionCtrl::init(SmplMsgConnection *connection, int robot_id)
{
  connection_ = connection;
  robot_id_ = robot_id;
  call_id_ = 0;
  return true;
}

bool MotomanMotionCtrl::controllerReady()
{
  std::string err_str;
  MotionReply reply;

  if (!sendAndReceive(MotionControlCmds::CHECK_MOTION_READY, reply))
  {
    ROS_ERROR("Failed to send CHECK_MOTION_READY command");
    return false;
  }

  return (reply.getResult() == MotionReplyResults::TRUE);
}

bool MotomanMotionCtrl::setTrajMode(bool enable)
{
  MotionReply reply;
  MotionControlCmd cmd = enable ? MotionControlCmds::START_TRAJ_MODE : MotionControlCmds::STOP_TRAJ_MODE;

  if (!sendAndReceive(cmd, reply))
  {
    ROS_ERROR("Failed to send TRAJ_MODE command");
    return false;
  }

  if (reply.getResult() != MotionReplyResults::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to set TrajectoryMode: " << getErrorString(reply));
    return false;
  }

  return true;
}

bool MotomanMotionCtrl::getMaxAcc(int groupNo, float *max_acc)
{
  MotionReply reply;
  MotionControlCmd cmd = MotionControlCmds::GET_MAX_ACC;

  if (!sendAndReceive(cmd, reply))
  {
    ROS_ERROR("Failed to send GET_MAX_ACC command");
    return false;
  }

  if (reply.getResult() != MotionReplyResults::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to set TrajectoryMode: " << getErrorString(reply));
    return false;
  }

  for (size_t i = 0; i < 8; ++i)
  {
    max_acc[i] = reply.getData(i);
  }

  return true;
}

bool MotomanMotionCtrl::setMaxAcc(int groupNo, float *max_acc)
{
  MotionReply reply;

  SimpleMessage req, res;
  MotionCtrl data;
  MotionCtrlMessage ctrl_msg;
  MotionReplyMessage ctrl_reply;

  data.init(groupNo, 0, MotionControlCmds::SET_MAX_ACC, 0);
  for (size_t i = 0; i < 8; ++i)
  {
    data.setData(i, max_acc[i]);
  }

  ctrl_msg.init(data);
  ctrl_msg.toRequest(req);

  if (!this->connection_->sendAndReceiveMsg(req, res))
  {
    ROS_ERROR("Failed to send MotionCtrl message");
    return false;
  }

  ctrl_reply.init(res);
  reply.copyFrom(ctrl_reply.reply_);

  if (reply.getResult() != MotionReplyResults::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to set TrajectoryMode: " << getErrorString(reply));
    return false;
  }

  return true;
}

bool MotomanMotionCtrl::setStreamMode(bool enable)
{
  ROS_ERROR("setStreamMode: %s", enable ? "true" : "false");

  MotionReply reply;
  MotionControlCmd cmd = enable ? MotionControlCmds::START_STREAM_MODE : MotionControlCmds::STOP_STREAM_MODE;

  if (!sendAndReceive(cmd, reply))
  {
    ROS_ERROR("Failed to send STREAM_MODE command");
    return false;
  }

  if (reply.getResult() != MotionReplyResults::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to set TrajectoryMode: " << getErrorString(reply));
    return false;
  }

  return true;
}

bool MotomanMotionCtrl::stopTrajectory()
{
  MotionReply reply;

  if (!sendAndReceive(MotionControlCmds::STOP_MOTION, reply))
  {
    ROS_ERROR("Failed to send STOP_MOTION command");
    return false;
  }

  if (reply.getResult() != MotionReplyResults::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to Stop Motion: " << getErrorString(reply));
    return false;
  }

  return true;
}

bool MotomanMotionCtrl::sendAndReceive(MotionControlCmd command, MotionReply &reply)
{
  SimpleMessage req, res;
  MotionCtrl data;
  MotionCtrlMessage ctrl_msg;
  MotionReplyMessage ctrl_reply;

  data.init(robot_id_, 0, command, 0);
  ctrl_msg.init(data);
  ctrl_msg.toRequest(req);
  {
    boost::mutex::scoped_lock lock(this->mutex_);
    if (!this->connection_->sendAndReceiveMsg(req, res))
    {
      ROS_ERROR("Failed to send MotionCtrl message");
      return false;
    }
  }

  ctrl_reply.init(res);
  reply.copyFrom(ctrl_reply.reply_);

  return true;
}

bool MotomanMotionCtrl::startJob(int taskNumber, std::string jobName, int &errorNumber)
{
  SimpleRpc data;
  SimpleRpcReply reply;
  data.init(call_id_, 0); //callid, no arguments
  const std::string str = "mpStartJob";

  data.setFunctionName(str);

  if (jobName.size() > 32)
  {
    ROS_ERROR("jobName is too long: %d", jobName.size());
    return false;
  }

  if (taskNumber < 0 || taskNumber > 15)
  {
    ROS_ERROR("Invalid task number: %d", taskNumber);
    return false;
  }

  MP_START_JOB_SEND_DATA sStartData;
  sStartData.sTaskNo = taskNumber;
  strcpy(sStartData.cJobName, jobName.c_str());
  const int field_length = sizeof(MP_START_JOB_SEND_DATA);
  std::vector<char> tmp_vector;
  tmp_vector.resize(field_length, 0);

  bswap(sStartData);
  memcpy(tmp_vector.data(), &sStartData, sizeof(MP_START_JOB_SEND_DATA));

  data.setArgumentsSize(tmp_vector.size());
  data.setArgumentsData(tmp_vector);

  if (!this->sendAndReceiveRpc(&data, &reply))
  {
    ROS_ERROR("failed to send mpStartJob command");
    return false;
  }
  call_id_++;
  const std::vector<char> buffer = reply.getResultData();
  const char *result_data = buffer.data();

  int status = reply.getStatus();

  if (status < 0)
  {
    ROS_ERROR("failed to get mpStartJob. Received status: %d", status);
    return false;
  }

  if (buffer.size() != sizeof(MP_STD_RSP_DATA))
  {
    ROS_ERROR("buffer size does not add up");
    return false;
  }

  MP_STD_RSP_DATA result;
  memcpy(&result, buffer.data(), buffer.size());
  bswap(result);
  errorNumber = result.err_no;
  return true;
}

bool MotomanMotionCtrl::deleteJob(std::string jobName, int &errorNumber)
{
  SimpleRpc data;
  SimpleRpcReply reply;
  data.init(call_id_, 0); //callid, no arguments
  const std::string str = "mpDeleteEnd";

  data.setFunctionName(str);

  MP_DELETE_JOB_SEND_DATA sData;
  strcpy(sData.cJobName, jobName.c_str());
  const int field_length = sizeof(MP_DELETE_JOB_SEND_DATA);
  std::vector<char> tmp_vector;
  tmp_vector.resize(field_length, 0);

  //bswap(sData);
  memcpy(tmp_vector.data(), &sData, sizeof(MP_DELETE_JOB_SEND_DATA));

  data.setArgumentsSize(tmp_vector.size());
  data.setArgumentsData(tmp_vector);

  if (!this->sendAndReceiveRpc(&data, &reply))
  {
    ROS_ERROR("failed to send mpDeleteEnd command");
    return false;
  }
  call_id_++;
  const std::vector<char> buffer = reply.getResultData();
  const char *result_data = buffer.data();

  int status = reply.getStatus();

  if (status < 0)
  {
    ROS_ERROR("failed to get mpDeleteEnd. Received status: %d", status);
    return false;
  }

  if (buffer.size() != sizeof(MP_STD_RSP_DATA))
  {
    ROS_ERROR("buffer size does not add up");
    return false;
  }

  MP_STD_RSP_DATA result;
  memcpy(&result, buffer.data(), buffer.size());
  bswap(result);
  errorNumber = result.err_no;

  return true;
}

bool MotomanMotionCtrl::waitForJobEnd(int taskNumber, int time, int &errorNumber)
{
  SimpleRpc data;
  SimpleRpcReply reply;
  data.init(call_id_, 0); //callid, no arguments
  const std::string str = "mpWaitForJobEnd";

  data.setFunctionName(str);

  MP_WAIT_JOB_SEND_DATA sWaitData;
  sWaitData.sTaskNo = taskNumber;
  sWaitData.sTime = time;
  const int field_length = sizeof(MP_WAIT_JOB_SEND_DATA);
  std::vector<char> tmp_vector;
  tmp_vector.resize(field_length, 0);

  bswap(sWaitData);
  memcpy(tmp_vector.data(), &sWaitData, sizeof(MP_WAIT_JOB_SEND_DATA));

  data.setArgumentsSize(tmp_vector.size());
  data.setArgumentsData(tmp_vector);

  if (!this->sendAndReceiveRpc(&data, &reply))
  {
    ROS_ERROR("failed to send mpWaitForJobEnd command");
    return false;
  }
  call_id_++;
  const std::vector<char> buffer = reply.getResultData();
  const char *result_data = buffer.data();

  int status = reply.getStatus();

  if (status < 0)
  {
    ROS_ERROR("failed to get mpWaitForJobEnd. Received status: %d", status);
    return false;
  }

  if (buffer.size() != sizeof(MP_STD_RSP_DATA))
  {
    ROS_ERROR("buffer size does not add up");
    return false;
  }

  MP_STD_RSP_DATA result;
  memcpy(&result, buffer.data(), buffer.size());
  bswap(result);
  errorNumber = result.err_no;

  return true;
}

bool MotomanMotionCtrl::setHold(int hold, int &errorNumber)
{
  SimpleRpc data;
  SimpleRpcReply reply;
  data.init(call_id_, 0); //callid, no arguments
  const std::string str = "mpHold";

  data.setFunctionName(str);

  MP_HOLD_SEND_DATA sHoldData;
  sHoldData.sHold = hold > 0 ? 1 : 0;
  const int field_length = sizeof(MP_HOLD_SEND_DATA);
  std::vector<char> tmp_vector;
  tmp_vector.resize(field_length, 0);

  bswap(sHoldData);
  memcpy(tmp_vector.data(), &sHoldData, sizeof(MP_HOLD_SEND_DATA));

  data.setArgumentsSize(tmp_vector.size());
  data.setArgumentsData(tmp_vector);

  if (!this->sendAndReceiveRpc(&data, &reply))
  {
    ROS_ERROR("failed to send mpHold command");
    return false;
  }
  call_id_++;
  const std::vector<char> buffer = reply.getResultData();
  const char *result_data = buffer.data();

  int status = reply.getStatus();

  if (status < 0)
  {
    ROS_ERROR("failed to get mpHold. Received status: %d", status);
    return false;
  }

  if (buffer.size() != sizeof(MP_STD_RSP_DATA))
  {
    ROS_ERROR("buffer size does not add up");
    return false;
  }

  MP_STD_RSP_DATA result;
  memcpy(&result, buffer.data(), buffer.size());
  bswap(result);
  errorNumber = result.err_no;

  return true;
}

bool MotomanMotionCtrl::getMasterJob(int taskNumber, std::string &jobName)
{
  SimpleRpc data;
  SimpleRpcReply reply;
  data.init(call_id_, 0); //callid, no arguments
  const std::string str = "mpGetMasterJob";

  data.setFunctionName(str);

  if (taskNumber < 0 || taskNumber > 15)
  {
    ROS_ERROR("Invalid task number: %d", taskNumber);
    return false;
  }

  MP_TASK_SEND_DATA sStartData;
  sStartData.sTaskNo = taskNumber;
  std::vector<char> tmp_vector(sizeof(MP_TASK_SEND_DATA), 0);

  bswap(sStartData);
  memcpy(tmp_vector.data(), &sStartData, sizeof(MP_TASK_SEND_DATA));

  data.setArgumentsSize(tmp_vector.size());
  data.setArgumentsData(tmp_vector);

  if (!this->sendAndReceiveRpc(&data, &reply))
  {
    ROS_ERROR("failed to send mpGetMasterJob command");
    return false;
  }
  call_id_++;
  const std::vector<char> buffer = reply.getResultData();
  const char *result_data = buffer.data();

  int status = reply.getStatus();

  if (status < 0)
  {
    ROS_ERROR("failed to get mpGetMasterJob. Received status: %d", status);
    return false;
  }

  if (buffer.size() != sizeof(MP_JOB_NAME_RSP_DATA))
  {
    ROS_ERROR("buffer size does not add up");
    return false;
  }

  MP_JOB_NAME_RSP_DATA result;
  memcpy(&result, buffer.data(), buffer.size());
  //bswap(result);
  jobName = std::string(result.cJobName);
  return true;
}


bool MotomanMotionCtrl::getCurJob(int taskNumber, int &jobLine, int &step, std::string &jobName)
{
  SimpleRpc data;
  SimpleRpcReply reply;
  data.init(call_id_, 0); //callid, no arguments
  const std::string str = "mpGetCurJob";

  data.setFunctionName(str);

  if (taskNumber < 0 || taskNumber > 15)
  {
    ROS_ERROR("Invalid task number: %d", taskNumber);
    return false;
  }

  MP_TASK_SEND_DATA sStartData;
  sStartData.sTaskNo = taskNumber;
  std::vector<char> tmp_vector(sizeof(MP_TASK_SEND_DATA), 0);

  bswap(sStartData);
  memcpy(tmp_vector.data(), &sStartData, sizeof(MP_TASK_SEND_DATA));

  data.setArgumentsSize(tmp_vector.size());
  data.setArgumentsData(tmp_vector);

  if (!this->sendAndReceiveRpc(&data, &reply))
  {
    ROS_ERROR("failed to send mpGetCurJob command");
    return false;
  }
  call_id_++;
  const std::vector<char> buffer = reply.getResultData();
  const char *result_data = buffer.data();

  int status = reply.getStatus();

  if (status < 0)
  {
    ROS_ERROR("failed to get mpGetCurJob. Received status: %d", status);
    return false;
  }

  if (buffer.size() != sizeof(MP_CUR_JOB_RSP_DATA))
  {
    ROS_ERROR("buffer size does not add up");
    return false;
  }

  MP_CUR_JOB_RSP_DATA result;
  memcpy(&result, buffer.data(), buffer.size());
  bswap(result);
  jobLine = result.usJobLine;
  step = result.usStep;
  jobName = std::string(result.cJobName);
  return true;
}

bool MotomanMotionCtrl::setCurJob(int jobLine, const std::string &jobName, int &errorNumber)
{
  SimpleRpc data;
  SimpleRpcReply reply;
  data.init(call_id_, 0); //callid, no arguments
  const std::string str = "mpSetCurJob";

  data.setFunctionName(str);

  if (jobName.size() > 32)
  {
    ROS_ERROR("jobName is too long: %d", jobName.size());
    return false;
  }

  MP_CUR_JOB_SEND_DATA sData;
  sData.usJobLine = jobLine;
  strcpy(sData.cJobName, jobName.c_str());
  std::vector<char> tmp_vector(sizeof(MP_CUR_JOB_SEND_DATA), 0);

  bswap(sData);
  memcpy(tmp_vector.data(), &sData, sizeof(MP_CUR_JOB_SEND_DATA));

  data.setArgumentsSize(tmp_vector.size());
  data.setArgumentsData(tmp_vector);

  if (!this->sendAndReceiveRpc(&data, &reply))
  {
    ROS_ERROR("failed to send mpSetCurJob command");
    return false;
  }
  call_id_++;
  const std::vector<char> buffer = reply.getResultData();
  const char *result_data = buffer.data();

  int status = reply.getStatus();

  if (status < 0)
  {
    ROS_ERROR("failed to get mpSetCurJob. Received status: %d", status);
    return false;
  }

  if (buffer.size() != sizeof(MP_CUR_JOB_RSP_DATA))
  {
    ROS_ERROR("buffer size does not add up");
    return false;
  }

  MP_STD_RSP_DATA result;
  memcpy(&result, buffer.data(), buffer.size());
  bswap(result);
  errorNumber = result.err_no;
  return true;
}

bool MotomanMotionCtrl::setMasterJob(int taskNumber, std::string jobName, int &errorNumber)
{
  SimpleRpc data;
  SimpleRpcReply reply;
  data.init(call_id_, 0); //callid, no arguments
  const std::string str = "mpSetMasterJob";

  data.setFunctionName(str);

  if (jobName.size() > 32)
  {
    ROS_ERROR("jobName is too long: %d", jobName.size());
    return false;
  }

  if (taskNumber < 0 || taskNumber > 15)
  {
    ROS_ERROR("Invalid task number: %d", taskNumber);
    return false;
  }

  MP_MASTER_JOB_SEND_DATA sData;
  sData.sTaskNo = taskNumber;
  strcpy(sData.cJobName, jobName.c_str());
  std::vector<char> tmp_vector(sizeof(MP_MASTER_JOB_SEND_DATA), 0);

  bswap(sData);
  memcpy(tmp_vector.data(), &sData, sizeof(MP_MASTER_JOB_SEND_DATA));

  data.setArgumentsSize(tmp_vector.size());
  data.setArgumentsData(tmp_vector);

  if (!this->sendAndReceiveRpc(&data, &reply))
  {
    ROS_ERROR("failed to send mpSetMasterJob command");
    return false;
  }
  call_id_++;
  const std::vector<char> buffer = reply.getResultData();
  const char *result_data = buffer.data();

  int status = reply.getStatus();

  if (status < 0)
  {
    ROS_ERROR("failed to get mpSetMasterJob. Received status: %d", status);
    return false;
  }

  if (buffer.size() != sizeof(MP_CUR_JOB_RSP_DATA))
  {
    ROS_ERROR("buffer size does not add up");
    return false;
  }

  MP_STD_RSP_DATA result;
  memcpy(&result, buffer.data(), buffer.size());
  bswap(result);
  errorNumber = result.err_no;
  return true;
}

bool MotomanMotionCtrl::cancelError(int &errorNumber)
{
  SimpleRpc data;
  SimpleRpcReply reply;
  data.init(call_id_, 0); //callid, no arguments
  const std::string str = "mpCancelError";

  data.setFunctionName(str);

  if (!this->sendAndReceiveRpc(&data, &reply))
  {
    ROS_ERROR("failed to send mpCancelError command");
    return false;
  }
  call_id_++;
  const std::vector<char> buffer = reply.getResultData();
  const char *result_data = buffer.data();

  int status = reply.getStatus();

  if (status < 0)
  {
    ROS_ERROR("failed to get mpCancelError. Received status: %d", status);
    return false;
  }

  const int funtion_name_size = 33;
  if (buffer.size() % funtion_name_size != 0)
  {
    ROS_ERROR("failed to get mpCancelError. Response size is invalid: %d", (int)buffer.size());
    return false;
  }

  MP_STD_RSP_DATA result;
  memcpy(&result, buffer.data(), buffer.size());
  bswap(result);
  errorNumber = result.err_no;
  return true;
}

bool MotomanMotionCtrl::resetAlarm(int &errorNumber)
{
  SimpleRpc data;
  SimpleRpcReply reply;
  data.init(call_id_, 0); //callid, no arguments
  const std::string str = "mpResetAlarm";

  data.setFunctionName(str);

  if (!this->sendAndReceiveRpc(&data, &reply))
  {
    ROS_ERROR("failed to send mpResetAlarm command");
    return false;
  }
  call_id_++;
  const std::vector<char> buffer = reply.getResultData();
  const char *result_data = buffer.data();

  int status = reply.getStatus();

  if (status < 0)
  {
    ROS_ERROR("failed to get mpResetAlarm. Received status: %d", status);
    return false;
  }

  const int funtion_name_size = 33;
  if (buffer.size() % funtion_name_size != 0)
  {
    ROS_ERROR("failed to get mpResetAlarm. Response size is invalid: %d", (int)buffer.size());
    return false;
  }

  MP_STD_RSP_DATA result;
  memcpy(&result, buffer.data(), buffer.size());
  bswap(result);
  errorNumber = result.err_no;
  return true;
}

bool MotomanMotionCtrl::listJobs(std::vector<std::string> &result)
{
  SimpleRpc data;
  SimpleRpcReply reply;
  data.init(call_id_, 0); //callid, no arguments
  const std::string str = "listJobs";

  data.setFunctionName(str);

  if (!this->sendAndReceiveRpc(&data, &reply))
  {
    ROS_ERROR("failed to send listJobs command");
    return false;
  }
  call_id_++;
  const std::vector<char> buffer = reply.getResultData();
  const char *result_data = buffer.data();

  int status = reply.getStatus();

  if (status < 0)
  {
    ROS_ERROR("failed to get listJobs. Received status: %d", status);
    return false;
  }

  const int funtion_name_size = 33;
  if (buffer.size() % funtion_name_size != 0)
  {
    ROS_ERROR("failed to get listJobs. Response size is invalid: %d", (int)buffer.size());
    return false;
  }

  const int count = buffer.size() / funtion_name_size;
  for (size_t i = 0; i < count; i++)
  {
    result.push_back(result_data);
    result_data += funtion_name_size;
  }

  return true;
}

bool MotomanMotionCtrl::getUserVars(motoman_msgs::GetUserVars::Request &req, motoman_msgs::GetUserVars::Response &res)
{
  SimpleRpc data;
  SimpleRpcReply reply;
  data.init(call_id_, 0); //callid, argumentssize
  const std::string str = "mpGetUserVars";

  data.setFunctionName(str);

  int countVariables = req.variables.size();

  for (size_t i = 0; i < countVariables; i++)
  {
    const motoman_msgs::UserVarPrimitive &prim = req.variables[i];
    MP_USR_VAR_INFO sData = {0};
    sData.var_type = prim.var_type;
    sData.var_no = prim.var_no;

    std::vector<char> tmp_vector(sizeof(MP_USR_VAR_INFO), 0);

    bswap(sData);
    memcpy(tmp_vector.data(), &sData, sizeof(MP_USR_VAR_INFO));

    data.setArgumentsSize(tmp_vector.size());
    data.setArgumentsData(tmp_vector);

    if (!this->sendAndReceiveRpc(&data, &reply))
    {
      ROS_ERROR("failed to send mpPutUserVars command");
      return false;
    }
    call_id_++;
    const std::vector<char> buffer = reply.getResultData();
    MP_USR_VAR_INFO *result_data = (MP_USR_VAR_INFO *)buffer.data();
    bswap(*result_data);

    int status = reply.getStatus();

    if (status < 0)
    {
      ROS_ERROR("failed to get mpPutUserVars. Received status: %d", status);
      res.success = false;
      res.err_no = status;
      return false;
    }
    else
    {
      res.success = true;
      res.err_no = status;
    }
    motoman_msgs::UserVarPrimitive resPrim;
    switch (result_data->var_type)
    {
    case 0:
      resPrim.int_value = result_data->val.b;
      break;
    case 1:
      resPrim.int_value = result_data->val.i;
      break;
    case 2:
      resPrim.int_value = result_data->val.d;
      break;
    case 3:
      resPrim.float_value = result_data->val.r;
      break;
    case 4:
      if (prim.string_value.size() > 16)
      {
        ROS_ERROR("String variable is too long");
        return false;
      }
      resPrim.string_value = result_data->val.s;
      break;
    default:
      ROS_ERROR("Unknown variable type");
      return false;
    }
    res.variables.push_back(resPrim);
  }

  return true;
}

bool MotomanMotionCtrl::putUserVars(motoman_msgs::PutUserVars::Request &req, motoman_msgs::PutUserVars::Response &res)
{
  SimpleRpc data;
  SimpleRpcReply reply;
  data.init(call_id_, 0); //callid, argumentssize
  const std::string str = "mpPutUserVars";

  data.setFunctionName(str);

  int countVariables = req.variables.size();

  for (size_t i = 0; i < countVariables; i++)
  {
    const motoman_msgs::UserVarPrimitive &prim = req.variables[i];
    MP_USR_VAR_INFO sData = {0};
    sData.var_type = prim.var_type;
    sData.var_no = prim.var_no;
    switch (prim.var_type)
    {
    case 0:
      sData.val.b = prim.int_value;
      break;
    case 1:
      sData.val.i = prim.int_value;
      break;
    case 2:
      sData.val.d = prim.int_value;
      break;
    case 3:
      sData.val.r = prim.float_value;
      break;
    case 4:
      if (prim.string_value.size() > 16)
      {
        ROS_ERROR("String variable is too long");
        return false;
      }
      strcpy(sData.val.s, prim.string_value.c_str());
      break;
    default:
      ROS_ERROR("Unknown variable type");
      return false;
    }

    std::vector<char> tmp_vector(sizeof(MP_USR_VAR_INFO), 0);

    bswap(sData);
    memcpy(tmp_vector.data(), &sData, sizeof(MP_USR_VAR_INFO));

    data.setArgumentsSize(tmp_vector.size());
    data.setArgumentsData(tmp_vector);

    if (!this->sendAndReceiveRpc(&data, &reply))
    {
      ROS_ERROR("failed to send mpPutUserVars command");
      return false;
    }
    call_id_++;
    const std::vector<char> buffer = reply.getResultData();
    const char *result_data = buffer.data();

    int status = reply.getStatus();

    if (status < 0)
    {
      ROS_ERROR("failed to get mpPutUserVars. Received status: %d", status);
      res.success = false;
      res.err_no = status;
      return false;
    }
    else
    {
      res.success = true;
      res.err_no = status;
    }
  }

  return true;
}

bool MotomanMotionCtrl::sendAndReceiveRpc(SimpleRpc *data, SimpleRpcReply *reply)
{
  SimpleRpcMessage ctrl_msg;
  SimpleRpcReplyMessage ctrl_reply;
  ctrl_msg.init(*data);
  SimpleMessage req, res;
  ctrl_msg.toRequest(req);
  boost::mutex::scoped_lock lock(this->mutex_);
  if (!this->connection_->sendAndReceiveMsg(req, res))
  {
    ROS_ERROR("Failed to send message");
    return false;
  }

  if (!ctrl_reply.init(res))
  {
    ROS_ERROR("Failed to decode message");
    return false;
  }
  reply->copyFrom(ctrl_reply.reply_);

  return true;
}

bool MotomanMotionCtrl::writeToIO(int address, int value)
{
  WriteSingleIO data;
  WriteSingleIOReply reply;
  WriteSingleIOMessage ctrl_msg;
  WriteSingleIOReplyMessage ctrl_reply;
  data.init(address, value);
  ctrl_msg.init(data);
  SimpleMessage req, res;
  ctrl_msg.toRequest(req);
  boost::mutex::scoped_lock lock(this->mutex_);
  if (!this->connection_->sendAndReceiveMsg(req, res))
  {
    ROS_ERROR("Failed to send message");
    return false;
  }
  ctrl_reply.init(res);
  reply.copyFrom(ctrl_reply.reply_);

  if (reply.getResultCode() != WriteSingleIOReplyResults::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to write to IO: " << getErrorString(reply));
    return false;
  }

  return true;
}

std::string MotomanMotionCtrl::getErrorString(const WriteSingleIOReply &reply)
{
  std::ostringstream ss;
  ss << reply.getResultString() << " (" << reply.getResultCode() << ")";
  return ss.str();
}

bool MotomanMotionCtrl::readFromIO(int address, int *value)
{
  ReadSingleIO data;
  ReadSingleIOReply reply;
  ReadSingleIOMessage ctrl_msg;
  ReadSingleIOReplyMessage ctrl_reply;
  data.init(address);
  ctrl_msg.init(data);
  SimpleMessage req, res;
  ctrl_msg.toRequest(req);
  boost::mutex::scoped_lock lock(this->mutex_);
  if (!this->connection_->sendAndReceiveMsg(req, res))
  {
    ROS_ERROR("Failed to send message");
    return false;
  }
  ctrl_reply.init(res);
  reply.copyFrom(ctrl_reply.reply_);

  if (reply.getResultCode() != ReadSingleIOReplyResults::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to read from IO: " << getErrorString(reply));
    return false;
  }

  *value = reply.getValue();
  return true;
}

std::string MotomanMotionCtrl::getErrorString(const ReadSingleIOReply &reply)
{
  std::ostringstream ss;
  ss << reply.getResultString() << " (" << reply.getResultCode() << ")";
  return ss.str();
}

std::string MotomanMotionCtrl::getErrorString(const MotionReply &reply)
{
  std::ostringstream ss;
  ss << reply.getResultString() << " (" << reply.getResult() << ")";
  ss << " : ";
  ss << reply.getSubcodeString() << " (" << reply.getSubcode() << ")";
  return ss.str();
}

} // namespace motion_ctrl
} // namespace motoman
