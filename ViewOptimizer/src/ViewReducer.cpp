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

#include <TMIV/Common/Common.h>
#include <TMIV/Renderer/reprojectPoints.h>
#include <TMIV/ViewOptimizer/ViewReducer.h>
#include <algorithm>
#include <cassert>
#include <cmath>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::Renderer;

namespace TMIV::ViewOptimizer {
ViewReducer::ViewReducer(const Json & /* node */) {}

auto ViewReducer::optimizeIntraPeriod(CameraParameterList cameras)
    -> Output<CameraParameterList> {
  Output<CameraParameterList> result;
  m_priorities.assign(cameras.size(), false);

  // choose 9 degree as quantization step of angle between view i and view j.
  const float degree_step = radperdeg * 9;

  size_t nbCameras = cameras.size();

  // search the largest angle between two views i,j
  vector<pair<size_t, size_t>> cameras_id_pair;
  pair<size_t, size_t> camera_id_pair;

  // Decide whether number of high priority list is one
  bool isoneview = false;

  size_t id_i;
  size_t id_j;

  // the range of view i and j
  float range_i = 0;
  float range_j = 0;

  // Early termination: if any view is full-ERP, choose this view
  for (auto id = 0u; id < nbCameras; id++) {
    if (cameras[id].type == ProjectionType::ERP) {
      if (abs(cameras[id].erpPhiRange[0] - cameras[id].erpPhiRange[1]) ==
          360.f) {
        if (abs(cameras[id].erpThetaRange[0] - cameras[id].erpThetaRange[1]) ==
            180.f) {
          for (size_t index = 0; index != cameras.size(); ++index) {
            (index == id ? result.base : result.additional)
                .push_back(move(cameras[index]));
          }

          m_priorities[id] = true;
          
          return result;
        }
      }
    }
  }

  // Calculate the angle between view i and view j
  size_t max_angle = 0;
  for (size_t id_1 = 0; id_1 < nbCameras - 1; id_1++) {
    for (size_t id_2 = id_1 + 1; id_2 < nbCameras; id_2++) {
      // Sphere distance function
      size_t temp_angle =
          size_t(acos(sin(cameras[id_1].rotation[1] * radperdeg) *
                   sin(cameras[id_2].rotation[1] * radperdeg) +
               cos(cameras[id_1].rotation[1] * radperdeg) *
                   cos(cameras[id_2].rotation[1] * radperdeg) *
                   cos((cameras[id_1].rotation[0] - cameras[id_2].rotation[0]) *
                       radperdeg)) /
          degree_step);

      if (temp_angle > max_angle) {
        cameras_id_pair.clear();
        cameras_id_pair.push_back(make_pair(id_1, id_2));
        max_angle = temp_angle;
      } else if (temp_angle == max_angle) {
        cameras_id_pair.push_back(make_pair(id_1, id_2));
      }
    }
  }
  // Early termination: if angle is zero, choose one view
  if (max_angle == 0) {
    isoneview = true;
  }

  // Calculate the sum of two views' FOV
  size_t max_FOV = 0;
  size_t max_num = 0;
  for (size_t id = 0; id < cameras_id_pair.size(); id++) {
    size_t temp_FOV = 0;
    size_t id_1 = cameras_id_pair[id].first;
    size_t id_2 = cameras_id_pair[id].second;

    temp_FOV = calculateFOV(cameras[id_1]) + calculateFOV(cameras[id_2]);

    if (temp_FOV > max_FOV) {
      max_FOV = temp_FOV;
      max_num = 1;
      swap(cameras_id_pair[0], cameras_id_pair[id]);
    } else if (temp_FOV == max_FOV) {
      swap(cameras_id_pair[max_num], cameras_id_pair[id]);
      max_num++;
    }
  }
  cameras_id_pair.erase(cameras_id_pair.begin() + max_num,
                        cameras_id_pair.end());
  // Select a pair of view which is farest to each other
  float max_distance = -1;
  for (size_t id = 0; id < cameras_id_pair.size(); id++) {
    float temp_distance = 0;
    size_t id_1 = cameras_id_pair[id].first;
    size_t id_2 = cameras_id_pair[id].second;

    temp_distance = calculateDistance(cameras[id_1], cameras[id_2]);
    if (temp_distance > max_distance) {
      camera_id_pair = cameras_id_pair[id];
      max_distance = temp_distance;
    }
  }

  // Calculte the overlap of view pair
  float overlap = 0;
  if (!isoneview) {
    id_i = camera_id_pair.first;
    id_j = camera_id_pair.second;

    // To do
    if (cameras[id_i].type == ProjectionType::ERP) {
      range_i =
          abs(cameras[id_i].erpPhiRange[0] - cameras[id_i].erpPhiRange[1]) *
          radperdeg;
    } else {
      range_i = atan(cameras[id_i].size[0] /
                     (2 * cameras[id_i].perspectiveFocal[0])) *
                radperdeg;
    }
    if (cameras[id_j].type == ProjectionType::ERP) {
      range_j =
          abs(cameras[id_j].erpPhiRange[0] - cameras[id_j].erpPhiRange[1]) *
          radperdeg;
    } else {
      range_j = atan(cameras[id_j].size[0] /
                     (2 * cameras[id_j].perspectiveFocal[0])) *
                radperdeg;
    }
    // Decide whether the number is one or multiple
    float overlapping =
        max(range_i - max_angle * degree_step, 0.f) +
        max(max_angle * degree_step + range_j - 360.f * radperdeg, 0.f);
    isoneview = (overlapping >= 0.5 * min(range_i, range_j)) ? true : false;
  }

  // Just select 1 view which has the shortest distance to center
  if (isoneview) {
    float x_center = 0;
    float y_center = 0;
    float z_center = 0;
    int id_center = 0;
    float distance = numeric_limits<float>::max();

    for (auto id = 0u; id < nbCameras; id++) {
      x_center += cameras[id].position[0];
      y_center += cameras[id].position[1];
      z_center += cameras[id].position[2];
    }
    x_center /= nbCameras;
    y_center /= nbCameras;
    z_center /= nbCameras;

    vector<size_t> camera_id;
    size_t max_FOV = 0;
    // Search views which have the largest FOV
    for (size_t id = 0; id < nbCameras; id++) {
      size_t temp_FOV = 0;

      temp_FOV = calculateFOV(cameras[id]);

      if (temp_FOV > max_FOV) {
        max_FOV = temp_FOV;
        camera_id.clear();
        camera_id.push_back(id);
      } else if (temp_FOV == max_FOV) {
        camera_id.push_back(id);
      }
    }

    for (auto i = 0u; i < camera_id.size(); i++) {
      float temp_distance =
          sqrtf(powf(cameras[camera_id[i]].position[0] - x_center, 2) +
                powf(cameras[camera_id[i]].position[1] - y_center, 2) +
                powf(cameras[camera_id[i]].position[2] - z_center, 2));
      if (temp_distance < distance) {
        id_center = int(camera_id[i]);
        distance = temp_distance;
      }
    }
    m_priorities[id_center] = true;
  }
  // Just select 2 view i and j
  else {
    m_priorities[camera_id_pair.first] = true;
    m_priorities[camera_id_pair.second] = true;
  }

  // Move cameras into base and additional partitions
  for (size_t index = 0; index != cameras.size(); ++index) {
    (m_priorities[index] ? result.base : result.additional)
        .push_back(move(cameras[index]));
  }
  return result;
}

auto ViewReducer::optimizeFrame(MVD16Frame views) const -> Output<MVD16Frame> {
  Output<MVD16Frame> result;
  assert(m_priorities.size() == views.size());

  // Move views into base and additional partitions
  for (size_t index = 0; index != views.size(); ++index) {
    (m_priorities[index] ? result.base : result.additional)
        .push_back(move(views[index]));
  }
  return result;
}
auto ViewReducer::calculateFOV(CameraParameters camera) -> size_t {
  size_t temp_FOV = 0;
  // choose 64 as quantization step of FOV.
  const float FOV_step = (45 * radperdeg) / 4;

  if (camera.type == ProjectionType::ERP) {
    temp_FOV += (size_t) (abs(camera.erpPhiRange[0] - camera.erpPhiRange[1]) *
                radperdeg *
                (abs(sin(camera.erpThetaRange[0] * radperdeg) -
                     sin(camera.erpThetaRange[1] * radperdeg))) /
                FOV_step);
  } else if (camera.type == ProjectionType::Perspective) {
    temp_FOV +=
        (size_t) (abs(
            4 * atan(camera.size[0] / (2 * camera.perspectiveFocal[0])) *
            sin(atan(camera.size[1] / (2 * camera.perspectiveFocal[1])))) /
        FOV_step);
  }
  return temp_FOV;
}
auto ViewReducer::calculateDistance(CameraParameters camera_1,
                                    CameraParameters camera_2) -> float {

  return sqrt(pow(camera_1.position[0] - camera_2.position[0], 2) +
              pow(camera_1.position[1] - camera_2.position[1], 2) +
              pow(camera_1.position[2] - camera_2.position[2], 2));
}
} // namespace TMIV::ViewOptimizer
