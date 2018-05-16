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

#ifndef MOTOMAN_DRIVER_MOTOMAN_SERVICES_H
#define MOTOMAN_DRIVER_MOTOMAN_SERVICES_H

#include <sstream>
#include <iostream>
#include <string>
#include <map>
#include "ros/ros.h"
#include "motoman_driver/motion_ctrl.h"
#include "std_srvs/Trigger.h"
#include "motoman_msgs/ListJobs.h"
#include "motoman_msgs/ReadIO.h"
#include "motoman_msgs/WriteIO.h"
#include "motoman_msgs/Hold.h"
#include "motoman_msgs/StartJob.h"
#include "motoman_msgs/WaitForJobEnd.h"
#include "motoman_msgs/SetMasterJob.h"
#include "motoman_msgs/SetCurJob.h"
#include "motoman_msgs/GetCurJob.h"
#include "motoman_msgs/ResetAlarm.h"
#include "motoman_msgs/GetMasterJob.h"
#include "motoman_msgs/DeleteJob.h"
#include "motoman_msgs/CancelError.h"
#include "motoman_msgs/PutUserVars.h"
#include "motoman_msgs/GetUserVars.h"

namespace motoman
{
namespace ros_services
{
using motoman::motion_ctrl::MotomanMotionCtrl;
class MotomanRosServices
{

  public:
    typedef boost::shared_ptr<MotomanRosServices> Ptr;
    static Ptr create(MotomanMotionCtrl *connection, ros::NodeHandle *pn);

    /**
     * \brief Destructor
     */
    ~MotomanRosServices();

  protected:
    /**
        * \brief Constructor
        *
        */
    MotomanRosServices(MotomanMotionCtrl *connection, ros::NodeHandle *pn);
    MotomanMotionCtrl *motion_ctrl_;
    ros::NodeHandle *node_;
    ros::ServiceServer srv_list_jobs_; // handle for job info service

    /**
   * \brief Service used to read io adresses in controller.
   */
    ros::ServiceServer srv_io_reader_;

    /**
   * \brief Service used to write value to io adresse in controller.
   */
    ros::ServiceServer srv_io_writer_;

    /**
   * \brief Service used to hold job in controller.
   */
    ros::ServiceServer srv_hold_;

    /**
   * \brief Service used to start job in controller.
   */
    ros::ServiceServer srv_startJob_;

    /**
   * \brief Service used to wait for job end in controller.
   */
    ros::ServiceServer srv_waitForJobEnd_;

    /**
   * \brief Service used to set master job in controller.
   */
    ros::ServiceServer srv_setMasterJob_;

    /**
   * \brief Service used to set current job in controller.
   */
    ros::ServiceServer srv_setCurJob_;

    /**
   * \brief Service used to set current job in controller.
   */
    ros::ServiceServer srv_getCurJob_;

    /**
   * \brief Service used to reset alarm in controller.
   */
    ros::ServiceServer srv_resetAlarm_;

    /**
   * \brief Service used to get Master job details in controller.
   */
    ros::ServiceServer srv_getMasterJob_;

    /**
   * \brief Service used to reset alarm in controller.
   */
    ros::ServiceServer srv_DeleteJob_;

    /**
   * \brief Service used to reset alarm in controller.
   */
    ros::ServiceServer srv_cancelError_;

    /**
   * \brief Service used to reset alarm in controller.
   */
    ros::ServiceServer srv_put_user_vars_;

    /**
   * \brief Service used to reset alarm in controller.
   */
    ros::ServiceServer srv_get_user_vars_;

    bool getUserVarsCB(motoman_msgs::GetUserVars::Request &req,
                       motoman_msgs::GetUserVars::Response &res);

    bool putUserVarsCB(motoman_msgs::PutUserVars::Request &req,
                       motoman_msgs::PutUserVars::Response &res);

    bool cancelErrorCB(motoman_msgs::CancelError::Request &req,
                       motoman_msgs::CancelError::Response &res);

    bool deleteJobCB(motoman_msgs::DeleteJob::Request &req,
                     motoman_msgs::DeleteJob::Response &res);

    bool getMasterJobCB(motoman_msgs::GetMasterJob::Request &req,
                        motoman_msgs::GetMasterJob::Response &res);

    bool resetAlarmCB(motoman_msgs::ResetAlarm::Request &req,
                      motoman_msgs::ResetAlarm::Response &res);

    bool setCurJobCB(motoman_msgs::SetCurJob::Request &req,
                     motoman_msgs::SetCurJob::Response &res);

    bool getCurJobCB(motoman_msgs::GetCurJob::Request &req,
                     motoman_msgs::GetCurJob::Response &res);

    bool setMasterJobCB(motoman_msgs::SetMasterJob::Request &req,
                        motoman_msgs::SetMasterJob::Response &res);

    bool waitForJobEndCB(motoman_msgs::WaitForJobEnd::Request &req,
                         motoman_msgs::WaitForJobEnd::Response &res);

    bool startJobCB(motoman_msgs::StartJob::Request &req,
                    motoman_msgs::StartJob::Response &res);

    bool holdCB(motoman_msgs::Hold::Request &req,
                motoman_msgs::Hold::Response &res);

    bool ioReadCB(motoman_msgs::ReadIO::Request &req,
                  motoman_msgs::ReadIO::Response &res);

    bool ioWriteCB(motoman_msgs::WriteIO::Request &req,
                   motoman_msgs::WriteIO::Response &res);

    bool listJobsCB(motoman_msgs::ListJobs::Request &req,
                    motoman_msgs::ListJobs::Response &res);
};
} // namespace ros_services
} // namespace motoman

#endif /* MOTOMAN_DRIVER_MOTOMAN_SERVICES_H */

//#include "motoman_msgs/CancelError.h"