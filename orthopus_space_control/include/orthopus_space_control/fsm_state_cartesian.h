/*
 *  fsm_state.h
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
#ifndef CARTESIAN_CONTROLLER_FSM_STATE_H
#define CARTESIAN_CONTROLLER_FSM_STATE_H

#include "ros/ros.h"

namespace space_control
{
// class FsmStateCartesian
// {
// public:
//   FsmState() : FsmState("Cartesian")
//   {
//   };
//
// protected:
// private:
//   ros::NodeHandle n_;
//
//   void entry_()
//   {
//     /* Switch to cartesian mode when position is completed */
//     cartesian_controller_.reset();
//     q_command_ = q_meas_;
//   };
//
//   void do_(){
//     ROS_INFO("=== Update joint position (Open loop)...");
//     q_current_ = q_command_;
//     ROS_INFO("    Done.");
//
//     cartesian_controller_.setDxDesired(dx_desired_);
//     cartesian_controller_.setInputSelector(CartesianController::INPUT_USER);
//     cartesian_controller_.run(q_current_, q_command_);
//
//     ROS_INFO("=== Send Niryo One commands...");
//     sendJointsCommand_();
//     ROS_INFO("    Done.");
//
//   };
// };
}
#endif
