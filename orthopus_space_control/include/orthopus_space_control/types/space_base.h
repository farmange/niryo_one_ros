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
#include "tf/tf.h"

// Eigen
#include "Eigen/Dense"

typedef Eigen::Matrix<double, 7, 1> Vector7d;

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
  SpaceBase() : position_(0, 0, 0), orientation_(0, 0, 0, 0){};
  virtual ~SpaceBase() = 0;

  /* Setter / Getter */
  void setX(double x)
  {
    position_(0) = x;
  };
  double getX() const
  {
    return position_(0);
  };

  void setY(double y)
  {
    position_(1) = y;
  };
  double getY() const
  {
    return position_(1);
  };

  void setZ(double z)
  {
    position_(2) = z;
  };
  double getZ() const
  {
    return position_(2);
  };

  double getQw() const
  {
    return orientation_.getW();
  };
  void setQw(double qw)
  {
    orientation_.setW(qw);
  };

  double getQx() const
  {
    return orientation_.getX();
  };
  void setQx(double qx)
  {
    orientation_.setX(qx);
  };

  double getQy() const
  {
    return orientation_.getY();
  };
  void setQy(double qy)
  {
    orientation_.setY(qy);
  };

  double getQz() const
  {
    return orientation_.getZ();
  };
  void setQz(double qz)
  {
    orientation_.setZ(qz);
  };

  Eigen::Vector3d getPosition() const
  {
    return position_;
  }
  void setPosition(const Eigen::Vector3d& p)
  {
    position_ = p;
  }

  tf::Quaternion getOrientation() const
  {
    return orientation_;
  }
  void setOrientation(const tf::Quaternion& q)
  {
    orientation_ = q;
  }

  Vector7d getRawVector()
  {
    Vector7d ret;
    ret(0) = position_(0);
    ret(1) = position_(1);
    ret(2) = position_(2);
    ret(3) = double(orientation_.getW());
    ret(4) = double(orientation_.getX());
    ret(5) = double(orientation_.getY());
    ret(6) = double(orientation_.getZ());
    return ret;
  }

  double operator[](int i) const
  {
    if (i >= 0 && i < 3)
    {
      return position_[i];
    }
    else if (i == 3)
    {
      return orientation_.w();
    }
    else if (i == 4)
    {
      return orientation_.x();
    }
    else if (i == 5)
    {
      return orientation_.y();
    }
    else if (i == 6)
    {
      return orientation_.z();
    }
  }

  /* ostream << operator */
  friend std::ostream& operator<<(std::ostream& os, const SpaceBase& sp)
  {
    return os << "position : [" << sp.position_(0) << ", " << sp.position_(1) << ", " << sp.position_(2)
              << "] orientation :[" << sp.orientation_.getW() << ", " << sp.orientation_.getX() << ", "
              << sp.orientation_.getY() << ", " << sp.orientation_.getZ() << "]";
  }

private:
  Eigen::Vector3d position_;
  tf::Quaternion orientation_;
};
}
#endif
