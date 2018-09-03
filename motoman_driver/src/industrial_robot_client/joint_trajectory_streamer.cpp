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

#include "motoman_driver/industrial_robot_client/joint_trajectory_streamer.h"
#include <map>
#include <vector>
#include <string>

namespace CommTypes = industrial::simple_message::CommTypes;
namespace ReplyTypes = industrial::simple_message::ReplyTypes;

namespace industrial_robot_client
{
namespace joint_trajectory_streamer
{

bool JointTrajectoryStreamer::init(SmplMsgConnection* connection, const std::map<int, RobotGroup> &robot_groups,
                                   const std::map<std::string, double> &velocity_limits)
{
  bool rtn = true;

  ROS_INFO("JointTrajectoryStreamer: init");

  rtn &= JointTrajectoryInterface::init(connection, robot_groups, velocity_limits);

  this->mutex_.lock();
  this->current_point_ = 0;
  this->state_ = TransferStates::IDLE;
  this->streaming_thread_ =
    new boost::thread(boost::bind(&JointTrajectoryStreamer::streamingThread, this));
  ROS_INFO("Unlocking mutex");
  this->mutex_.unlock();

  return rtn;
}

bool JointTrajectoryStreamer::init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                                   const std::map<std::string, double> &velocity_limits)
{
  bool rtn = true;

  ROS_INFO("JointTrajectoryStreamer: init");

  rtn &= JointTrajectoryInterface::init(connection, joint_names, velocity_limits);

  this->mutex_.lock();
  this->current_point_ = 0;
  this->state_ = TransferStates::IDLE;
  this->streaming_thread_ =
    new boost::thread(boost::bind(&JointTrajectoryStreamer::streamingThread, this));
  ROS_INFO("Unlocking mutex");
  this->mutex_.unlock();

  return rtn;
}

JointTrajectoryStreamer::~JointTrajectoryStreamer()
{
  delete this->streaming_thread_;
}

void JointTrajectoryStreamer::jointTrajectoryCB(const motoman_msgs::DynamicJointTrajectoryConstPtr &msg)
{
  ROS_INFO("Receiving joint trajectory message");

  // read current state value (should be atomic)
  int state = this->state_;

  ROS_DEBUG("Current state is: %d", state);
  if (TransferStates::IDLE != state)
  {
    if (msg->points.empty())
      ROS_INFO("Empty trajectory received, canceling current trajectory");
    else
      ROS_ERROR("Trajectory splicing not yet implemented, stopping current motion.");

    this->mutex_.lock();
    trajectoryStop();
    this->mutex_.unlock();
    return;
  }

  if (msg->points.empty())
  {
    ROS_INFO("Empty trajectory received while in IDLE state, nothing is done");
    return;
  }

  // calc new trajectory
  std::vector<SimpleMessage> new_traj_msgs;
  if (!trajectory_to_msgs(msg, &new_traj_msgs))
    return;

  // send command messages to robot
  send_to_robot(new_traj_msgs);
}

void JointTrajectoryStreamer::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  ROS_INFO("Receiving joint trajectory message");

  // read current state value (should be atomic)
  int state = this->state_;

  ROS_DEBUG("Current state is: %d", state);
  if (TransferStates::IDLE != state)
  {
    if (msg->points.empty())
      ROS_INFO("Empty trajectory received, canceling current trajectory");
    else
      ROS_ERROR("Trajectory splicing not yet implemented, stopping current motion.");

    this->mutex_.lock();
    trajectoryStop();
    this->mutex_.unlock();
    return;
  }

  if (msg->points.empty())
  {
    ROS_INFO("Empty trajectory received while in IDLE state, nothing is done");
    return;
  }

  // calc new trajectory
  std::vector<SimpleMessage> new_traj_msgs;
  if (!trajectory_to_msgs(msg, &new_traj_msgs))
    return;

  // send command messages to robot
  send_to_robot(new_traj_msgs);
}

void JointTrajectoryStreamer::jointCommandCB(
  const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  // read current state value (should be atomic)
  int state = this->state_;

  ROS_DEBUG("Current state is: %d", state);

  //If current state is idle, set to POINT_STREAMING
  if (TransferStates::IDLE == state)
  {
    this->mutex_.lock();
      this->state_ = TransferStates::POINT_STREAMING;
      this->current_point_ = 0;
      this->streaming_sequence_ = 0;
      this->streaming_start_ = ros::Time::now();
      this->streaming_queue_ = std::queue<SimpleMessage>();
      this->setStreamingMode(true);
      state = TransferStates::POINT_STREAMING;
      time_since_last_ = 0.0;
      time_of_last_ = ros::Time::now().toSec();
    this->mutex_.unlock();
    ROS_INFO("First joint point received. Starting on-the-fly streaming.");
  }

  //if current state is POINT_STREAMING, process incoming point.
  if (TransferStates::POINT_STREAMING == state)
  {
    if (msg->points.empty())
    {
      ROS_INFO("Empty point received, cancelling current trajectory");
      return;
    }

    //Else, Push point into queue
    SimpleMessage message;

    if (!create_message(this->streaming_sequence_, *msg, &message))
      return;

    /*trajectory_msgs::JointTrajectoryPoint rbt_pt, xform_pt;

    // select / reorder joints for sending to robot
    if (!select(msg->joint_names, msg->points[0], this->all_joint_names_, &rbt_pt))
      return;

    // transform point data (e.g. for joint-coupling)
    if (!transform(rbt_pt, &xform_pt))
      return;

    // convert trajectory point to ROS message
    if (!create_message(this->streaming_sequence_, xform_pt, &message))
      return; */

    //Points get pushed into queue here. They will be popped in the Streaming Thread and sent to controller.
    this->mutex_.lock();
      while (!this->streaming_queue_.empty())   // ## clear queue
      {
        this->streaming_queue_.pop();
      }

      this->streaming_queue_.push(message);
      this->streaming_sequence_++;
    this->mutex_.unlock();
   }
   //Else, cannot splice. Cancel current motion.
  else
  {
    if (msg->points.empty())
      ROS_INFO("Empty trajectory received, canceling current trajectory");
    else
      ROS_ERROR("Trajectory splicing not yet implemented, stopping current motion.");

	  this->mutex_.lock();
      trajectoryStop();
	  this->mutex_.unlock();
    return;
  }
}

bool JointTrajectoryStreamer::send_to_robot(const std::vector<SimpleMessage>& messages)
{
  boost::mutex::scoped_lock lock(this->mutex_);
  int state = this->state_;
  ROS_INFO("Loading trajectory, setting state to streaming");
  if (TransferStates::IDLE == state || TransferStates::POINT_STREAMING == state)
  {
    ROS_WARN_COND(TransferStates::POINT_STREAMING == state, "Trajectory execution request received. Abort POINT_STREAMING_MODE");
    ROS_INFO("Executing trajectory of size: %d", static_cast<int>(messages.size()));
    this->current_traj_ = messages;
    this->current_point_ = 0;
    this->state_ = TransferStates::STREAMING;
    this->setStreamingMode(false);
    this->streaming_start_ = ros::Time::now();
  }
  else
  {
    ROS_ERROR_STREAM("TransferState is not IDLE and not in POINT_STREAMING state.");
    return false;
  }

  return true;
}

bool JointTrajectoryStreamer::trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr &traj, std::vector<SimpleMessage>* msgs)
{
  ROS_INFO("JointTrajectoryStreamer::trajectory_to_msgs A");
  // use base function to transform points
  if (!JointTrajectoryInterface::trajectory_to_msgs(traj, msgs))
    return false;

  // pad trajectory as required for minimum streaming buffer size
  if (!msgs->empty() && (msgs->size() < (size_t)min_buffer_size_))
  {
    ROS_DEBUG("Padding trajectory: current(%d) => minimum(%d)", static_cast<int>(msgs->size()), min_buffer_size_);
    while (msgs->size() < (size_t)min_buffer_size_)
      msgs->push_back(msgs->back());
  }

  return true;
}

bool JointTrajectoryStreamer::trajectory_to_msgs(const motoman_msgs::DynamicJointTrajectoryConstPtr &traj, std::vector<SimpleMessage>* msgs)
{
  ROS_INFO("JointTrajectoryStreamer::trajectory_to_msgs B");
  // use base function to transform points
  if (!JointTrajectoryInterface::trajectory_to_msgs(traj, msgs))
    return false;

  // pad trajectory as required for minimum streaming buffer size
  if (!msgs->empty() && (msgs->size() < (size_t)min_buffer_size_))
  {
    ROS_DEBUG("Padding trajectory: current(%d) => minimum(%d)", static_cast<int>(msgs->size()), min_buffer_size_);
    while (msgs->size() < (size_t)min_buffer_size_)
      msgs->push_back(msgs->back());
  }

  return true;
}


void JointTrajectoryStreamer::streamingThread()
{
  int connectRetryCount = 1;

  ROS_INFO("Starting joint trajectory streamer thread");
  while (ros::ok())
  {
    ros::Duration(0.005).sleep();

    // automatically re-establish connection, if required
    if (connectRetryCount-- > 0)
    {
      ROS_INFO("Connecting to robot motion server");
      this->connection_->makeConnect();
      ros::Duration(0.250).sleep();  // wait for connection

      if (this->connection_->isConnected())
        connectRetryCount = 0;
      else if (connectRetryCount <= 0)
      {
        ROS_ERROR("Timeout connecting to robot controller.  Send new motion command to retry.");
        this->state_ = TransferStates::IDLE;
      }
      continue;
    }

    this->mutex_.lock();

    SimpleMessage msg, tmpMsg, reply;

    switch (this->state_)
    {
    case TransferStates::IDLE:
      ros::Duration(0.01).sleep();  //  slower loop while waiting for new trajectory
      break;

    case TransferStates::STREAMING:
      ROS_INFO("Industrial TransferStates::STREAMING");
      if (this->current_point_ >= static_cast<int>(this->current_traj_.size()))
      {
        ROS_INFO("Trajectory streaming complete, setting state to IDLE");
        this->state_ = TransferStates::IDLE;
        break;
      }

      if (!this->connection_->isConnected())
      {
        ROS_DEBUG("Robot disconnected.  Attempting reconnect...");
        connectRetryCount = 5;
        break;
      }

      tmpMsg = this->current_traj_[this->current_point_];
      msg.init(tmpMsg.getMessageType(), CommTypes::SERVICE_REQUEST,
               ReplyTypes::INVALID, tmpMsg.getData());  // set commType=REQUEST

      ROS_DEBUG("Sending joint trajectory point");
      if (this->connection_->sendAndReceiveMsg(msg, reply, false))
      {
        ROS_DEBUG("Point[%d of %d] sent to controller",
                 this->current_point_, static_cast<int>(this->current_traj_.size()));
        this->current_point_++;
      }
      else
        ROS_WARN("Failed sent joint point, will try again");

      break;
   case TransferStates::POINT_STREAMING:

        ROS_DEBUG("POINT_STREAMING in base JointTrajectoryStreamer");   // ##

        //if no points in queue, streaming complete, set to idle.
        if (this->streaming_queue_.empty())
        {
          ROS_INFO("Point streaming complete, setting state to IDLE");
          this->state_ = TransferStates::IDLE;
          break;
        }

        //if not connected, reconnect.
        if (!this->connection_->isConnected())
        {
          ROS_WARN("Robot disconnected.  Attempting reconnect...");
          connectRetryCount = 5;
          break;
        }

        //otherwise, send point to robot.
        tmpMsg = this->streaming_queue_.front();
        this->streaming_queue_.pop();
        msg.init(tmpMsg.getMessageType(), CommTypes::SERVICE_REQUEST,
                 ReplyTypes::INVALID, tmpMsg.getData());  // set commType=REQUEST

        ROS_DEBUG("Sending joint trajectory point");
        if (this->connection_->sendAndReceiveMsg(msg, reply, false))
        {
          ROS_DEBUG("Point[%d] sent to controller", this->current_point_);
          this->current_point_++;
        }
        else
          ROS_WARN("Failed sent joint point, will try again");

        break;
        // consider checking for controller point starvation here. use a timer to check if the state
        // is popping in and out of POINT_STREAMING mode, indicating something is trying to send streaming
        // points, but is doing so too slowly. It may, in fact, not matter other than motion won't be smooth.

    default:
      ROS_ERROR("Joint trajectory streamer: unknown state, %d", this->state_);
      this->state_ = TransferStates::IDLE;
      break;
    }

    this->mutex_.unlock();
  }

  ROS_WARN("Exiting trajectory streamer thread");
}

void JointTrajectoryStreamer::trajectoryStop()
{
  JointTrajectoryInterface::trajectoryStop();

  ROS_DEBUG("Stop command sent, entering idle mode");
  this->state_ = TransferStates::IDLE;
}

}  // namespace joint_trajectory_streamer
}  // namespace industrial_robot_client

