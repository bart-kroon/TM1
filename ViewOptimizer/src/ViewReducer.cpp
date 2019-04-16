/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <TMIV/Renderer/reprojectPoints.h>
#include <TMIV/ViewOptimizer/ViewReducer.h>
#include <algorithm>
#include <cassert>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::Renderer;

namespace TMIV::ViewOptimizer {
ViewReducer::ViewReducer(const Json & /* node */) {}

auto ViewReducer::optimizeIntraPeriod(Metadata::CameraParameterList cameras)
    -> Output<Metadata::CameraParameterList> {
  Output<Metadata::CameraParameterList> result;
  m_priorities.assign(cameras.size(), false);

  const float radperdeg = 0.01745329251994329576923690768489f;

  int nbCameras = (int)cameras.size();

  // search the largest angle between two views i,j
  int id_i = 0;
  int id_j = 0;
  // the range of view i and j
  float range_i = 0;
  float range_j = 0;

  float max_angle = 0;
  for (int id_1 = 0u; id_1 < nbCameras - 1; id_1++) {
    for (int id_2 = id_1 + 1; id_2 < nbCameras; id_2++) {

      // Sphere distance function
      float temp_angle =
          acos(sin(cameras[id_1].rotation[1] * radperdeg) *
                   sin(cameras[id_2].rotation[1] * radperdeg) +
               sin(cameras[id_1].rotation[1] * radperdeg) *
                   sin(cameras[id_2].rotation[1] * radperdeg) *
                   cos((cameras[id_1].rotation[0] - cameras[id_2].rotation[0]) *
                       radperdeg));

      if (temp_angle > max_angle) {
        id_i = id_1;
        id_j = id_2;
        max_angle = temp_angle;
      }
    }
  }
  // Calculte the hor_range of view i and j
  if (cameras[id_i].type == ProjectionType::ERP) {
    range_i = abs(cameras[id_i].erpPhiRange[0] - cameras[id_i].erpPhiRange[1]) *
              radperdeg;
  } else {
    range_i =
        atan(cameras[id_i].size[0] / (2 * cameras[id_i].perspectiveFocal[0])) *
        radperdeg;
  }
  if (cameras[id_j].type == ProjectionType::ERP) {
    range_j = abs(cameras[id_j].erpPhiRange[0] - cameras[id_j].erpPhiRange[1]) *
              radperdeg;
  } else {
    range_j =
        atan(cameras[id_j].size[0] / (2 * cameras[id_j].perspectiveFocal[0])) *
        radperdeg;
  }
  // Decide whether the number is one or multiple
  float overlapping = max(range_i - max_angle, 0.f) +
                      max(max_angle + range_j - 360.f * radperdeg, 0.f);

  // Just select 1 view which has the shortest distance to center
  if (overlapping > 0.25 * (range_j + range_i)) {

    float x_center = 0;
    float y_center = 0;
    float z_center = 0;
    int id_center = 0;
    float distance = std::numeric_limits<float>::max();

    for (int id = 0u; id < nbCameras; id++) {
      x_center += cameras[id].position[0];
      y_center += cameras[id].position[1];
      z_center += cameras[id].position[2];
    }
    x_center /= nbCameras;
    y_center /= nbCameras;
    z_center /= nbCameras;

    for (int id = 0u; id < nbCameras; id++) {
      float distance_temp = sqrtf(powf(cameras[id].position[0] - x_center, 2) +
                                  powf(cameras[id].position[1] - y_center, 2) +
                                  powf(cameras[id].position[2] - z_center, 2));
      if (distance_temp < distance) {
        id_center = id;
        distance = distance_temp;
      }
    }
    m_priorities[id_center] = true;
  }
  // Just select 2 view i and j
  else {
    m_priorities[id_i] = true;
    m_priorities[id_j] = true;
  }

  // Move cameras into base and additional partitions
  for (size_t index = 0; index != cameras.size(); ++index) {
    (m_priorities[index] ? result.base : result.additional)
        .push_back(move(cameras[index]));
  }
  return result;
}

auto ViewReducer::optimizeFrame(Common::MVD16Frame views) const
    -> Output<Common::MVD16Frame> {
  Output<Common::MVD16Frame> result;
  assert(m_priorities.size() == views.size());

  // Move views into base and additional partitions
  for (size_t index = 0; index != views.size(); ++index) {
    (m_priorities[index] ? result.base : result.additional)
        .push_back(move(views[index]));
  }
  return result;
}
} // namespace TMIV::ViewOptimizer
