/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Fraunhofer IPA
 * Author: Thiago de Freitas
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
 *  * Neither the name of the Fraunhofer IPA, nor the names
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

#include "motoman_driver/industrial_robot_client/motoman_emergencs_stop_ros_service.h"


int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "emergency_stop_node");
  ros::NodeHandle pn("~");
  int freq;
  ros::param::param<int>("~frequency", freq, 10);
  ros::Rate loop_rate(freq);

  int alarm_code;
  ros::param::param<int>("~alarm_code", alarm_code, 8053);

  int sub_code;
  ros::param::param<int>("~sub_code", sub_code, 5);

  std::string root_name;
  ros::param::param<std::string>("~robot_service_root_name", root_name, "/sda10d");

  std::string message;
  ros::param::param<std::string>("~alarm_message", message, "emergency");

  motoman::ros_services::MotomanEmergencyStopRosService::Ptr service_ptr = motoman::ros_services::MotomanEmergencyStopRosService::create(&pn);

  service_ptr->setTopicServiceRoot(root_name);
  service_ptr->setAlarmMessage(message);
  service_ptr->setSubCode(sub_code);
  service_ptr->setAlarmCode(alarm_code);

  while (ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}

