
/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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

#ifndef MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_ROBOT_FEEDBACK_MONITOR_H
#define MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_ROBOT_FEEDBACK_MONITOR_H

#include <vector>
#include <string>
namespace industrial_robot_client
{
namespace joint_trajectory_action
{
class RobotFeedbackMonitor
{
  public:
    RobotFeedbackMonitor(){};

    void init(std::vector<std::string> jnames);

    std::vector<double> get_joint_values()
    {
        this->reset_updates();
        return this->values_;
    }

    std::vector<std::string> get_joint_names()
    {
        return this->joint_names_;
    }

    void set_joint_names(std::vector<std::string> jnames)
    {
        this->joint_names_ = jnames;
    }

    std::vector<double> select(const std::vector<std::string> & jnames);

    void update_joint_values(const std::vector<std::string> & jnames, const std::vector<double> & values);

    bool all_updated();
    void reset_updates();

  protected:
    std::vector<std::string> joint_names_;
    std::vector<double> values_;
    std::vector<bool> updated_;
    bool is_valid();
};
}
}
#endif //  MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_ROBOT_FEEDBACK_MONITOR_H
