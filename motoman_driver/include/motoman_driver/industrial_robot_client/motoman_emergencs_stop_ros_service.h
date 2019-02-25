/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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

#ifndef MOTOMAN_DRIVER_MOTOMAN_E_SERVICE_H
#define MOTOMAN_DRIVER_MOTOMAN_E_SERVICE_H

#include <sstream>
#include <iostream>
#include <string>
#include <map>
#include "ros/ros.h"
#include "std_srvs/Trigger.h"


namespace motoman
{
namespace ros_services
{
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
  timeout = 0xffffffff
};

static std::string printErrorCode(int error_no)
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
class MotomanEmergencyStopRosService
{

public:
  typedef boost::shared_ptr<MotomanEmergencyStopRosService> Ptr;
  static Ptr create(ros::NodeHandle *pn);
  void setAlarmMessage(std::string message){ this->alarm_message = message;};
  void setSubCode(int value){ this->sub_code = value;};
  void setAlarmCode(int value){ this->alarm_code = value;};
  /**
     * \brief Destructor
     */
  ~MotomanEmergencyStopRosService();

protected:
  /**
        * \brief Constructor
        *
        */
  MotomanEmergencyStopRosService(ros::NodeHandle *pn);
  ros::NodeHandle *node_;
  int alarm_code;
  int sub_code;
  std::string alarm_message;
  bool disableRobot();
  bool switchOffServoPower();
  bool setAlarm();
  /**
  * \brief Service to trigger emergency stop
  */
  ros::ServiceServer srv_trigger_e_stop_;

  bool triggerEstopCB(std_srvs::Trigger::Request &req,
                      std_srvs::Trigger::Response &res);

};
} // namespace ros_services
} // namespace motoman

#endif /* MOTOMAN_DRIVER_MOTOMAN_E_SERVICE_H */

//#include "motoman_msgs/CancelError.h"