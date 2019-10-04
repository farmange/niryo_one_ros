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

namespace cartesian_controller
{
class RobotManagerFsmState
{
public:
  enum FsmStateEnum
  {
    Disable = 0,
    GotoHome,
    GotoRest,
    GotoDrink,
    GotoStandGlass,
    FlipPinch,
    CartesianMode,
    Idle
  };

  RobotManagerFsmState() = default;
  const std::string ToString() const
  {
    std::string msg;
    switch (state)
    {
      case CartesianMode:
        msg = "CartesianMode";
        break;
      case GotoHome:
        msg = "GotoHome";
        break;
      case GotoRest:
        msg = "GotoRest";
        break;
      case GotoDrink:
        msg = "GotoDrink";
        break;
      case GotoStandGlass:
        msg = "GotoStandGlass";
        break;
      case FlipPinch:
        msg = "FlipPinch";
        break;
      case Disable:
        msg = "Disable";
        break;
      case Idle:
        msg = "Idle";
        break;
      default:
        msg = "NaN";
        break;
    }
    return msg;
  }

  RobotManagerFsmState& operator=(const FsmStateEnum s)
  {
    state = s;
  };
  friend bool operator==(const RobotManagerFsmState& s, const FsmStateEnum& e)
  {
    return s.state == e;
  };
  friend bool operator!=(const RobotManagerFsmState& s, const FsmStateEnum& e)
  {
    return s.state != e;
  };
  friend bool operator==(const RobotManagerFsmState& s1, const RobotManagerFsmState& s2)
  {
    return s1.state == s2.state;
  };
  friend bool operator!=(const RobotManagerFsmState& s1, const RobotManagerFsmState& s2)
  {
    return s1.state != s2.state;
  };

private:
  FsmStateEnum state;
};

class RobotManagerFsmAction
{
public:
  enum FsmActionEnum
  {
    None = 0,
    Cartesian,
    GotoHome,
    GotoRest,
    GotoDrink,
    GotoStandGlass,
    FlipPinch
  };

  RobotManagerFsmAction() = default;
  const std::string ToString() const
  {
    std::string msg;
    switch (action)
    {
      case None:
        msg = "None";
        break;
      case Cartesian:
        msg = "Cartesian";
        break;
      case GotoHome:
        msg = "GotoHome";
        break;
      case GotoRest:
        msg = "GotoRest";
        break;
      case GotoDrink:
        msg = "GotoDrink";
        break;
      case GotoStandGlass:
        msg = "GotoStandGlass";
        break;
      case FlipPinch:
        msg = "FlipPinch";
        break;
      default:
        msg = "NaN";
        break;
    }
    return msg;
  }
  RobotManagerFsmAction& operator=(const FsmActionEnum a)
  {
    action = a;
  }
  friend bool operator==(const RobotManagerFsmAction& a, const FsmActionEnum& e)
  {
    return a.action == e;
  };
  friend bool operator!=(const RobotManagerFsmAction& a, const FsmActionEnum& e)
  {
    return a.action == e;
  };

private:
  FsmActionEnum action;
};
}

#endif
