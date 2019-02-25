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

#include <std_srvs/Trigger.h>
#include "motoman_msgs/SetAlarm.h"
#include "motoman_msgs/SetServoPower.h"
#include "motoman_driver/industrial_robot_client/motoman_emergencs_stop_ros_service.h"


namespace motoman
{
namespace ros_services
{

MotomanEmergencyStopRosService::MotomanEmergencyStopRosService(ros::NodeHandle *pn)
    : node_(pn), alarm_code(8053), sub_code(5), alarm_message("emergency")
{
  srv_trigger_e_stop_ = node_->advertiseService("trigger_emergency_stop", &MotomanEmergencyStopRosService::triggerEstopCB, this);
}

MotomanEmergencyStopRosService::~MotomanEmergencyStopRosService()
{
}

MotomanEmergencyStopRosService::Ptr MotomanEmergencyStopRosService::create(ros::NodeHandle *pn)
{
  return MotomanEmergencyStopRosService::Ptr(new MotomanEmergencyStopRosService(pn));
}

bool MotomanEmergencyStopRosService::disableRobot()
{
  ros::ServiceClient disable_robot_client = node_->serviceClient<std_srvs::Trigger>("~disable_robot");
  std_srvs::Trigger srv;
  if (disable_robot_client.call(srv))
  {
    ROS_ERROR("Failed to call service disable_robot");
    return false;
  }
  return true;
}

bool MotomanEmergencyStopRosService::switchOffServoPower()
{
  ros::ServiceClient set_servo_power_client = node_->serviceClient<motoman_msgs::SetServoPower>("~set_servo_power");
  motoman_msgs::SetServoPower srv;
  srv.request.power_on = false;
  if (set_servo_power_client.call(srv))
  {
    ROS_ERROR("Failed to call service to set servo power.");
    return false;
  }
  //ROS_INFO(srv.response.message);
  return true;
}

bool MotomanEmergencyStopRosService::setAlarm()
{
  ros::ServiceClient set_alarm_client = node_->serviceClient<motoman_msgs::SetAlarm>("~set_alarm");
  motoman_msgs::SetAlarm srv;
  srv.request.alm_code = this->alarm_code;
  srv.request.sub_code = this->sub_code;
  srv.request.alm_msg = this->alarm_message;
  if (set_alarm_client.call(srv))
  {
    ROS_ERROR("Failed to call service to set servo power.");
    return false;
  }
  //ROS_INFO(srv.response.message);
  return true;
}

bool MotomanEmergencyStopRosService::triggerEstopCB(std_srvs::Trigger::Request &req,
                                                    std_srvs::Trigger::Response &res)
{
  res.success = true;
  bool success = false;
  success = this->disableRobot();
  if (!success)
  {
    res.message = "could not disable robot";
    res.success = false;
    return true;
  }
  success = this->switchOffServoPower();
  if (!success)
  {
    res.message = "could not switch off servo power";
    res.success = false;
    return true;
  }
  success = this->setAlarm();
  if (!success)
  {
    res.message = "could not set alarm on robot panel";
    res.success = false;
    return true;
  }
  return true;
}

} // namespace ros_services
} // namespace motoman
