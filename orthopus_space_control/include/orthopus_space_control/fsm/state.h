/*
 *  state.h
 *  Copyright (C) 2019 Orthopus
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef CARTESIAN_CONTROLLER_STATE_H
#define CARTESIAN_CONTROLLER_STATE_H

#include "ros/ros.h"

namespace space_control
{
template <class T>
class State
{
public:
  State(T* obj, std::string name)
  {
    ROS_DEBUG("Construct state %s", name.c_str());
    name_ = name;
    obj_ = obj;
    enter_ptr_ = nullptr;
    update_ptr_ = nullptr;
    exit_ptr_ = nullptr;

    if (obj_ == nullptr)
    {
      ROS_ERROR("No state machine obj !");
      return;
    }
  };

  ~State(){};

  void enter()
  {
    if (obj_ == nullptr)
    {
      ROS_ERROR("No state machine context object !");
      return;
    }
    if (enter_ptr_ == nullptr)
    {
      //       ROS_DEBUG("No ENTER function for the state %s !", name_.c_str());
      return;
    }
    ROS_DEBUG("Run enter function of state '%s'...", name_.c_str());
    (obj_->*enter_ptr_)();
  };

  void update()
  {
    if (obj_ == nullptr)
    {
      ROS_ERROR("No state machine context object !");
      return;
    }
    if (update_ptr_ == nullptr)
    {
      //       ROS_DEBUG("No UPDATE function for the state %s !", name_.c_str());
      return;
    }
    ROS_DEBUG("Run update function of state '%s'...", name_.c_str());
    (obj_->*update_ptr_)();
  };

  void exit()
  {
    if (obj_ == nullptr)
    {
      ROS_ERROR("No state machine context object !");
      return;
    }
    if (exit_ptr_ == nullptr)
    {
      //       ROS_DEBUG("No EXIT function for the state %s !", name_.c_str());
      return;
    }
    ROS_DEBUG("Run exit function of state '%s'...", name_.c_str());
    (obj_->*exit_ptr_)();
  };

  void registerEnterFcn(void (T::*fp)(void))
  {
    enter_ptr_ = fp;
  };

  void registerUpdateFcn(void (T::*fp)(void))
  {
    ROS_DEBUG("registerUpdateFcn");
    update_ptr_ = fp;
  };

  void registerExitFcn(void (T::*fp)(void))
  {
    exit_ptr_ = fp;
  };

  std::string getName()
  {
    return name_;
  };

private:
  T* obj_;
  std::string name_;
  void (T::*enter_ptr_)(void);
  void (T::*update_ptr_)(void);
  void (T::*exit_ptr_)(void);
};
}
#endif
