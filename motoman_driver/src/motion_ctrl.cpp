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

#include "ros/ros.h"
#include "simple_message/simple_message.h"
#include <string>

namespace MotionControlCmds = motoman::simple_message::motion_ctrl::MotionControlCmds;
namespace MotionReplyResults = motoman::simple_message::motion_reply::MotionReplyResults;
namespace ReadSingleIOReplyResults = motoman::simple_message::io_ctrl_reply::ReadSingleIOReplyResults;
namespace WriteSingleIOReplyResults = motoman::simple_message::io_ctrl_reply::WriteSingleIOReplyResults;
using motoman::simple_message::motion_ctrl::MotionCtrl;
using motoman::simple_message::motion_ctrl_message::MotionCtrlMessage;
using motoman::simple_message::motion_reply_message::MotionReplyMessage;

using motoman::simple_message::io_ctrl::ReadSingleIO;
using motoman::simple_message::io_ctrl_reply::ReadSingleIOReply;
using motoman::simple_message::io_ctrl_reply_message::ReadSingleIOReplyMessage;
using motoman::simple_message::io_ctrl_message::ReadSingleIOMessage;

using motoman::simple_message::io_ctrl::WriteSingleIO;
using motoman::simple_message::io_ctrl_reply::WriteSingleIOReply;
using motoman::simple_message::io_ctrl_reply_message::WriteSingleIOReplyMessage;
using motoman::simple_message::io_ctrl_message::WriteSingleIOMessage;

using industrial::simple_message::SimpleMessage;

namespace motoman
{
namespace motion_ctrl
{

bool MotomanMotionCtrl::init(SmplMsgConnection* connection, int robot_id)
{
  connection_ = connection;
  robot_id_ = robot_id;
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


bool MotomanMotionCtrl::getMaxAcc(int groupNo, float* max_acc)
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


bool MotomanMotionCtrl::setMaxAcc(int groupNo, float* max_acc)
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
  ROS_ERROR("setStreamMode: %s", enable ? "true": "false");

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

  if (!this->connection_->sendAndReceiveMsg(req, res))
  {
    ROS_ERROR("Failed to send MotionCtrl message");
    return false;
  }

  ctrl_reply.init(res);
  reply.copyFrom(ctrl_reply.reply_);

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


bool MotomanMotionCtrl::readFromIO(int address, int* value)
{
  ReadSingleIO data;
  ReadSingleIOReply reply;
  ReadSingleIOMessage ctrl_msg;
  ReadSingleIOReplyMessage ctrl_reply;
  data.init(address);
  ctrl_msg.init(data);
  SimpleMessage req, res;
  ctrl_msg.toRequest(req);
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


}  // namespace motion_ctrl
}  // namespace motoman

