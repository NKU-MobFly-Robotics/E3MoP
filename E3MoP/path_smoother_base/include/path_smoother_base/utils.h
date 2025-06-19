/******************************************************************************
 * Copyright (c) 2022, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#pragma once

#include <vector>

#include "geometry_msgs/Pose.h"

namespace path_smoother_base {

struct MapInfo {
 public:
  bool operator==(const MapInfo& map_info) const {
    if (size_x != map_info.size_x || size_y != map_info.size_y ||
        resolution != map_info.resolution ||
        origin.position.x != map_info.origin.position.x ||
        origin.position.y != map_info.origin.position.y) {
      return false;
    } else {
      return true;
    }
  }

  bool operator!=(const MapInfo& map_info) const {
    return !(*this == map_info);
  }

  uint32_t size_x;
  uint32_t size_y;
  float resolution;
  geometry_msgs::Pose origin;
};

struct EuclidDistGrid {
  MapInfo info;
  std::vector<double> data;
};

inline double distance(double x0, double y0, double x1, double y1) {
  return hypot(x1 - x0, y1 - y0);
}

inline double distanceToLine(double pX, double pY, double x0, double y0,
                             double x1, double y1) {
  double A = pX - x0;
  double B = pY - y0;
  double C = x1 - x0;
  double D = y1 - y0;

  double dot = A * C + B * D;
  double len_sq = C * C + D * D;
  double param = dot / len_sq;

  double xx, yy;

  if (param < 0.0) {
    xx = x0;
    yy = y0;
  } else if (param > 1.0) {
    xx = x1;
    yy = y1;
  } else {
    xx = x0 + param * C;
    yy = y0 + param * D;
  }

  return distance(pX, pY, xx, yy);
}

void calcInscribedAndCircumscribedRadius(
    const std::vector<geometry_msgs::Point>& footprint,
    double* inscribed_radius_m, double* circumscribed_radius_m);

}  // namespace path_smoother_base
