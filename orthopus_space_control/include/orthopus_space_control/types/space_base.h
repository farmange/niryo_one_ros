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

#include "geometry_msgs/Pose.h"

// Eigen
#include "Eigen/Dense"

typedef Eigen::Matrix<double, 7, 1> Vector7d;

namespace space_control
{
/**
* \brief Parent class of coordinate space position or velocity.
*/
class SpaceBase
{
public:
  SpaceBase() : position(0, 0, 0), orientation(0, 0, 0, 0){};

  SpaceBase(double (&raw_data)[7])
  {
    for (int i = 0; i < 7; i++)
    {
      (*this)[i] = raw_data[i];
    }
  };

  SpaceBase(const geometry_msgs::Pose p)
  {
    position.x() = p.position.x;
    position.y() = p.position.y;
    position.z() = p.position.z;
    orientation.w() = p.orientation.w;
    orientation.x() = p.orientation.x;
    orientation.y() = p.orientation.y;
    orientation.z() = p.orientation.z;
  };

  virtual ~SpaceBase() = 0;

  class Positiond : public Eigen::Vector3d
  {
  public:
    using Eigen::Vector3d::Vector3d;
    inline double x(void) const
    {
      return m_storage.data()[0];
    };
    inline double y(void) const
    {
      return m_storage.data()[1];
    };
    inline double z(void) const
    {
      return m_storage.data()[2];
    };

    inline double& x(void)
    {
      return m_storage.data()[0];
    };
    inline double& y(void)
    {
      return m_storage.data()[1];
    };
    inline double& z(void)
    {
      return m_storage.data()[2];
    };
  };

  class Orientationd : public Eigen::Quaterniond
  {
  public:
    using Eigen::Quaterniond::Quaterniond;
    Eigen::Vector4d toVector() const
    {
      Eigen::Vector4d vec;
      vec(0) = w();
      vec(1) = x();
      vec(2) = y();
      vec(3) = z();
      return vec;
    };
  };

  bool isAllZero() const
  {
    bool ret = true;
    for (int i = 0; i < 7; i++)
    {
      if ((*this)[i] != 0.0)
      {
        ret = false;
        break;
      }
    }
    return ret;
  };

  Positiond getPosition() const
  {
    return position;
  };
  void setPosition(const Positiond& p)
  {
    position = p;
  };

  Orientationd getOrientation() const
  {
    return orientation;
  };

  void setOrientation(const Orientationd& q)
  {
    orientation = q;
  };

  Vector7d getRawVector() const
  {
    Vector7d ret;
    ret(0) = position.x();
    ret(1) = position.y();
    ret(2) = position.z();
    ret(3) = orientation.w();
    ret(4) = orientation.x();
    ret(5) = orientation.y();
    ret(6) = orientation.z();
    return ret;
  };

  /* Write raw data operator */
  double& operator[](int i)
  {
    if (i >= 0 && i < 3)
    {
      return position[i];
    }
    else if (i == 3)
    {
      return orientation.w();
    }
    else if (i == 4)
    {
      return orientation.x();
    }
    else if (i == 5)
    {
      return orientation.y();
    }
    else if (i == 6)
    {
      return orientation.z();
    }
  };

  /* Read raw data operator */
  double operator[](int i) const
  {
    if (i >= 0 && i < 3)
    {
      return position[i];
    }
    else if (i == 3)
    {
      return orientation.w();
    }
    else if (i == 4)
    {
      return orientation.x();
    }
    else if (i == 5)
    {
      return orientation.y();
    }
    else if (i == 6)
    {
      return orientation.z();
    }
  };

  /* ostream << operator */
  friend std::ostream& operator<<(std::ostream& os, const SpaceBase& sp)
  {
    return os << "position : [" << sp.position(0) << ", " << sp.position(1) << ", " << sp.position(2)
              << "] orientation :[" << sp.orientation.w() << ", " << sp.orientation.x() << ", " << sp.orientation.y()
              << ", " << sp.orientation.z() << "]";
  };

  Positiond position;
  Orientationd orientation;
};
}
#endif
