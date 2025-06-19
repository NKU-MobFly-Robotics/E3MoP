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

#include "path_smoother_base/utils.h"

#include <cmath>

namespace path_smoother_base {

void calcInscribedAndCircumscribedRadius(
    const std::vector<geometry_msgs::Point>& footprint,
    double* inscribed_radius_m, double* circumscribed_radius_m) {
  *inscribed_radius_m = std::numeric_limits<double>::max();
  *circumscribed_radius_m = 0.0;

  if (footprint.size() <= 2) {
    return;
  }

  for (size_t i = 0; i < footprint.size() - 1; ++i) {
    // check the distance from the robot center point to the first vertex
    double vertex_dist = distance(0.0, 0.0, footprint[i].x, footprint[i].y);
    double edge_dist = distanceToLine(0.0, 0.0, footprint[i].x, footprint[i].y,
                                      footprint[i + 1].x, footprint[i + 1].y);
    *inscribed_radius_m =
        std::min({*inscribed_radius_m, vertex_dist, edge_dist});
    *circumscribed_radius_m =
        std::max({*circumscribed_radius_m, vertex_dist, edge_dist});
  }

  // we also need to do the last vertex and the first vertex
  double vertex_dist =
      distance(0.0, 0.0, footprint.back().x, footprint.back().y);
  double edge_dist =
      distanceToLine(0.0, 0.0, footprint.back().x, footprint.back().y,
                     footprint.front().x, footprint.front().y);
  *inscribed_radius_m = std::min({*inscribed_radius_m, vertex_dist, edge_dist});
  *circumscribed_radius_m =
      std::max({*circumscribed_radius_m, vertex_dist, edge_dist});
}

}  // namespace path_smoother_base
