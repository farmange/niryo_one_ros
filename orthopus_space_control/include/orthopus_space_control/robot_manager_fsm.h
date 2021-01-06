/*
 *  robot_manager_fsm.h
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
#ifndef CARTESIAN_CONTROLLER_ROBOT_MANAGER_FSM_H
#define CARTESIAN_CONTROLLER_ROBOT_MANAGER_FSM_H

namespace space_control
{
class FsmInputEvent
{
public:
  enum FsmInputEventEnum
  {
    None = 0,
    JointHome,
    JointRest,
    TrajectoryDrink,
    TrajectoryStand
  };

  FsmInputEvent() = default;

  const std::string toString() const
  {
    std::string msg;
    switch (input_event_)
    {
      case None:
        msg = "None";
        break;
      case JointHome:
        msg = "JointHome";
        break;
      case JointRest:
        msg = "JointRest";
        break;
      case TrajectoryDrink:
        msg = "TrajectoryDrink";
        break;
      case TrajectoryStand:
        msg = "TrajectoryStand";
        break;
      default:
        msg = "NaN";
        break;
    }
    return msg;
  }
  FsmInputEvent& operator=(const FsmInputEventEnum e)
  {
    input_event_ = e;
  }
  friend bool operator==(const FsmInputEvent& c, const FsmInputEventEnum& e)
  {
    return c.input_event_ == e;
  };
  friend bool operator!=(const FsmInputEvent& c, const FsmInputEventEnum& e)
  {
    return c.input_event_ == e;
  };

private:
  FsmInputEventEnum input_event_;
};
}

#endif
