/*
 *  space_base.h
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
#ifndef CARTESIAN_CONTROLLER_SPACE_BASE_H
#define CARTESIAN_CONTROLLER_SPACE_BASE_H

#include "ros/ros.h"

namespace cartesian_controller
{
class SpaceBase : public std::vector<double>
{ 
public:
  enum PositionIndex
  {
    kX = 0,
    kY = 1,
    kZ = 2
  };

  /** Summarises all possible logical values. */
  enum EulerIndex
  {
    kRoll = 3, /**< Logical value for "false". */
    kPitch = 4,
    kYaw = 5
  };

  enum QuatIndex
  {
    kQw = 3,
    kQx = 4,
    kQy = 5,
    kQz = 6
  };

  SpaceBase(bool use_quaternion) : use_quaternion_(use_quaternion)
  {
    if (use_quaternion_)
    {
      resize(7, 0.0);
    }
    else
    {
      resize(6, 0.0);
    }
  }

  /* Overload ostream << operator */
  friend std::ostream& operator<<(std::ostream& os, const SpaceBase& sp)
  {
    using namespace std;
    copy(sp.begin(), sp.end(), ostream_iterator<double>(os, ", "));
    return os;
  }

  bool getUseQuaternion()
  {
    return use_quaternion_;
  }
  
private:
  bool    use_quaternion_;
  virtual void abstract_function_() = 0;
};
}
#endif
