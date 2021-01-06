/*
 *  engine.h
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
#ifndef CARTESIAN_CONTROLLER_ENGINE_H
#define CARTESIAN_CONTROLLER_ENGINE_H

#include "ros/ros.h"

#include "orthopus_space_control/fsm/state.h"
#include "orthopus_space_control/fsm/transition.h"

namespace space_control
{
template <class T>
class Engine
{
public:
  Engine(T* obj)
  {
    ROS_DEBUG("Construct FSM engine");
    obj_ = obj;
    current_state_ == nullptr;
    if (obj_ == nullptr)
    {
      ROS_ERROR("Bad state machine context object !");
      return;
    }
  };

  ~Engine(){};

  void registerState(State<T>* state)
  {
    states_.push_back(state);
  };

  void registerTransition(Transition<T>* transition)
  {
    transitions_.push_back(transition);
  };

  void process()
  {
    if (current_state_ == nullptr)
    {
      ROS_ERROR("Undefined current state !");
      return;
    }
    bool found = false;
    ROS_DEBUG("Looking for transition for the current state '%s'", current_state_->getName().c_str());

    for (int i = 0; i < transitions_.size(); i++)
    {
      if (found == true)
      {
        break;
      }
      std::vector<State<T>*> init_state = transitions_[i]->getInitialStates();
      for (int j = 0; j < init_state.size(); j++)
      {
        if (init_state[j] == current_state_)
        {
          ROS_DEBUG("Found transition to go from '%s' to '%s' state...", current_state_->getName().c_str(),
                    transitions_[i]->getFinalState()->getName().c_str());
          if (transitions_[i]->isConditionFulfilled())
          {
            ROS_ERROR("=> Condition is fulfil ! Go to '%s'", transitions_[i]->getFinalState()->getName().c_str());
            current_state_->exit();
            current_state_ = transitions_[i]->getFinalState();
            current_state_->enter();
            found = true;
            break;
          }
        }
      }
    }
    current_state_->update();
  };

  void setCurrentState(State<T>* state)
  {
    current_state_ = state;
  };

  State<T>* getCurrentState()
  {
    return current_state_;
  };

private:
  T* obj_;
  State<T>* current_state_;
  std::vector<State<T>*> states_;
  std::vector<Transition<T>*> transitions_;
};
}
#endif
