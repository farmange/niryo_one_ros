/*
 *  event.h
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
#ifndef CARTESIAN_CONTROLLER_EVENT_H
#define CARTESIAN_CONTROLLER_EVENT_H

#include "ros/ros.h"

namespace cartesian_controller
{
enum class Event
{
  None = 0,
//   Cartesian,
//   GotoHome,
//   GotoRest,
//   GotoDrink,
//   GotoStandGlass,
//   FlipPinch
  Disable,
  Idle,
  ExecuteJointTraj,
  ExecuteSpaceControl,
  ExecuteSpaceTraj
};
}
#endif
