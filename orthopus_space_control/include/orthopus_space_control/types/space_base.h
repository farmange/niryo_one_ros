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

namespace space_control
{
/**
* \brief Parent class of coordinate space position or velocity. The vector size depends on orientation definition.
* The vector size could be :
*       - 6 in case of euler (roll, pitch and yaw) representation of the orientation
*       - 7 in case of quaternion (Qw, Qx, Qy, Qz) representation of the orientation
*/
class SpaceBase : public std::vector<double>
{
public:
  /** Position part of the vector */
  enum PositionIndex
  {
    kX = 0, /**< X component */
    kY = 1, /**< Y component */
    kZ = 2  /**< Z component */
  };

  /** Orientation part of the vector (Euler form) */
  enum EulerIndex
  {
    kRoll = 3,  /**< Roll component */
    kPitch = 4, /**< Pitch component */
    kYaw = 5    /**< Yaw component */
  };

  /** Orientation part of the vector (quaternion form) */
  enum QuatIndex
  {
    kQw = 3, /**< Scalar part of the quaternion */
    kQx = 4, /**< X component of vector part of the quaternion */
    kQy = 5, /**< Y component of vector part of the quaternion */
    kQz = 6  /**< Z component of vector part of the quaternion */
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

  virtual ~SpaceBase() = 0;

  bool getUseQuaternion()
  {
    return use_quaternion_;
  }

  /* Overload ostream << operator */
  friend std::ostream& operator<<(std::ostream& os, const SpaceBase& sp)
  {
    using namespace std;
    copy(sp.begin(), sp.end(), ostream_iterator<double>(os, ", "));
    return os;
  }

private:
  bool use_quaternion_;
};
}
#endif
