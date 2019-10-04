/*
 *  joint_position.h
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
#ifndef CARTESIAN_CONTROLLER_JOINT_POSITION_H
#define CARTESIAN_CONTROLLER_JOINT_POSITION_H

#include "ros/ros.h"

namespace cartesian_controller
{
class JointPosition : public std::vector<double>
{
private:
  int joint_number_;

public:
  //      using vector::operator=;

  JointPosition() : joint_number_(0)
  {
    resize(0);
  };

  JointPosition(int joint_number) : joint_number_(joint_number)
  {
    resize(joint_number, 0.0);
  };

  /* Overload ostream << operator */
  friend std::ostream& operator<<(std::ostream& os, const JointPosition& sp)
  {
    using namespace std;
    copy(sp.begin(), sp.end(), ostream_iterator<double>(os, ", "));
    return os;
  };

  //      /* Overload ostream << operator */
  //      friend std::ostream &operator << (std::ostream &os, const JointPosition &sp) {
  //        using namespace std;
  //        copy(sp.begin(), sp.end(), ostream_iterator<double>(os, "\n"));
  //        return os;
  //      }

  int getJointNumber()
  {
    return joint_number_;
  };
};
}
#endif
