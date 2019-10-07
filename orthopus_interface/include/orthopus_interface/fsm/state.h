/*
 *  robot_state.h
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

#include "orthopus_interface/fsm/event.h"

namespace cartesian_controller
{
  class RobotManager;
 
class State
{
public:
  State() {};
  virtual ~State() {};
  virtual State* handleInput(RobotManager& robot, Event event) {};
  virtual void update(RobotManager& robot) {};
  virtual void enter(RobotManager& robot) {};
  virtual void exit(RobotManager& robot) {};
};


}
#endif
