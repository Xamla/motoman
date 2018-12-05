
#include <algorithm>
#include <stdexcept>
#include <ros/ros.h>
#include "motoman_driver/industrial_robot_client/robot_feedback_monitor.h"

namespace industrial_robot_client
{
  namespace joint_trajectory_action
  {
    void RobotFeedbackMonitor::init(std::vector<std::string> jnames)
    {
      this->joint_names_ = jnames;
      for (int i = 0; i < jnames.size(); i++)
      {
        this->values_.push_back(0.0);
        this->updated_.push_back(false);
      }
    }

    //Whenever values are read from monitor the values are marked as not updated
    std::vector<double> RobotFeedbackMonitor::select(const std::vector<std::string> & jnames, bool reset_updated)
    {
      boost::mutex::scoped_lock lock( this->mutex );
      std::vector<double> result;
      for (int i = 0; i < jnames.size(); i++)
      {
        std::vector<std::string>::iterator it;
        it = std::find(this->joint_names_.begin(), this->joint_names_.end(), jnames[i]);
        if (it != this->joint_names_.end())
        {
          int index = std::distance(this->joint_names_.begin(), it);
          result.push_back(this->values_[index]);
          if (reset_updated)
          {
            this->updated_[index] = false;
          }
        }
        else
        {
          throw std::invalid_argument("joint value is not in robot state monitor");
        }
      }
      return result;
    }

    void RobotFeedbackMonitor::update_joint_values(const std::vector<std::string> & jnames, const std::vector<double> & values)
    {
      assert(this->is_valid() && jnames.size() == values.size());
      boost::mutex::scoped_lock lock( this->mutex );
      for (int i = 0; i < jnames.size(); i++)
      {
        std::vector<std::string>::iterator it;
        it = std::find(this->joint_names_.begin(), this->joint_names_.end(), jnames[i]);
        if (it != this->joint_names_.end())
        {
          int index = std::distance(this->joint_names_.begin(), it);
          this->values_[index] = values[i];
          this->updated_[index] = true;
        }
        else
        {
          throw std::invalid_argument("joint value is not in robot state monitor");
        }
      }
    }

    bool RobotFeedbackMonitor::all_updated()
    {
      boost::mutex::scoped_lock lock( this->mutex );
      for (int i = 0; i < this->updated_.size(); i++)
      {
        if (!this->updated_[i])
        {
          return false;
        }
      }
      return true;
    }

    void RobotFeedbackMonitor::reset_updates()
    {
      boost::mutex::scoped_lock lock( this->mutex );
      for (int i = 0; i < updated_.size(); i++)
      {
        this->updated_[i] = false;
      }
    }

    bool RobotFeedbackMonitor::is_valid()
    {
      boost::mutex::scoped_lock lock( this->mutex );
      return this->updated_.size() == this->joint_names_.size() && this->values_.size() == this->joint_names_.size();
    }
  }
}
