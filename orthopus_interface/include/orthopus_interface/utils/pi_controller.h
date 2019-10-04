/*
 *  pi_controller.h
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
#ifndef CARTESIAN_CONTROLLER_PI_CONTROLLER_H
#define CARTESIAN_CONTROLLER_PI_CONTROLLER_H

namespace cartesian_controller
{
class PiController
{
public:
  PiController()
  {
    pi_max_ = 0.0;
    pi_min_ = 0.0;
    p_gain_ = 0.0;
    i_gain_ = 0.0;
    i_sum_ = 0.0;
    pi_out_ = 0.0;
  };

  void setGains(const double& p_gain, const double& i_gain)
  {
    p_gain_ = p_gain;
    i_gain_ = i_gain;
  };

  void init(double sampling_period, double pi_min = 1.0, double pi_max = 1.0)
  {
    sampling_period_ = sampling_period;
    pi_max_ = pi_max;
    pi_min_ = pi_min;
  };

  void reset()
  {
    i_sum_ = 0.0;
    pi_out_ = 0.0;
  };

  void execute(const double& error, double& output)
  {
    bool int_ok = true;
    /* Proportional term */
    double proportional_out = p_gain_ * error;
    /* Integral term */
    double i_sum_temp_ = i_sum_ + (error * sampling_period_);
    double integral_out = i_gain_ * i_sum_temp_;
    /* PI */
    pi_out_ = proportional_out + integral_out;

    // Restrict to max/min
    if (pi_out_ > pi_max_)
    {
      pi_out_ = pi_max_;

      /* Error is the same sign? Inhibit integration. */
      if (error > 0)
      {
        int_ok = false;
      }
    }
    else if (pi_out_ < pi_min_)
    {
      pi_out_ = pi_min_;

      /* Error is the same sign? Inhibit integration. */
      if (error < 0)
      {
        int_ok = false;
      }
    }

    /* Update the integrator if allowed. */
    if (int_ok)
    {
      i_sum_ = i_sum_temp_;
    }
    output = pi_out_;
  };

protected:
private:
  double sampling_period_;
  double pi_min_, pi_max_;
  double p_gain_;
  double i_gain_;
  double i_sum_;
  double pi_out_;
};
}
#endif
