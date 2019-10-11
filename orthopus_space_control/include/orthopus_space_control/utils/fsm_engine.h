/*
 *  fsm_engine.h
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
#ifndef CARTESIAN_CONTROLLER_FSM_ENGINE_H
#define CARTESIAN_CONTROLLER_FSM_ENGINE_H

#include "ros/ros.h"

#include <orthopus_space_control/utils/fsm_state.h>
#include <orthopus_space_control/utils/fsm_transition.h>

namespace space_control
{
class FsmEngine
{
public:
  FsmEngine(){};
  void processFsm(){

  };

  void addState(FsmState state)
  {
    state_.push_back(state);
  };

  void addTransition(FsmTransition transition)
  {
    transition_.push_back(transition);
  };

protected:
private:
  //   std::vector<FsmState> state_;
  //   std::vector<FsmTransition> transition_;
};
}
#endif
