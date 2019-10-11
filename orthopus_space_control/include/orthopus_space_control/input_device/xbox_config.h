/*
 *  xbox_config.h
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
#ifndef CARTESIAN_CONTROLLER_XBOX_CONFIG_H
#define CARTESIAN_CONTROLLER_XBOX_CONFIG_H

// http://wiki.ros.org/joy
// Microsoft Xbox 360 Wired Controller for Linux
//
// Table of index number of /joy.buttons:
// 0 : A
// 1 : B
// 2 : X
// 3 : Y
// 4 : LB
// 5 : RB
// 6 : back
// 7 : start
// 8 : power
// 9 : Button stick left
// 10 : Button stick right
//
// Table of index number of /joy.axes:
// 0 : Left/Right Axis stick left
// 1 : Up/Down Axis stick left
// 2 : LT
// 3 : Left/Right Axis stick right
// 4 : Up/Down Axis stick right
// 5 : RT
// 6 : cross key left/right
// 7 : cross key up/down

#define XBOX_BUTTON_A 0
#define XBOX_BUTTON_B 1
#define XBOX_BUTTON_X 2
#define XBOX_BUTTON_Y 3
#define XBOX_BUTTON_LB 4
#define XBOX_BUTTON_RB 5
#define XBOX_BUTTON_BACK 6
#define XBOX_BUTTON_START 7
#define XBOX_BUTTON_POWER 8
#define XBOX_BUTTON_STICK_LEFT 9
#define XBOX_BUTTON_STICK_RIGHT 10

#define XBOX_AXIS_HORIZONTAL_LEFT 0
#define XBOX_AXIS_VERTICAL_LEFT 1
#define XBOX_LT 2
#define XBOX_AXIS_HORIZONTAL_RIGHT 3
#define XBOX_AXIS_VERTICAL_RIGHT 4
#define XBOX_RT 5
#define XBOX_CROSS_HORIZONTAL 6
#define XBOX_CROSS_VERTICAL 7

#endif
