/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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
 *  * Neither the name of the ISO/IEC nor the names of its contributors may
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
#include <TMIV/Image/Image.h>
#include <TMIV/Renderer/reprojectPoints.h>
#include <TMIV/ViewOptimizer/ViewReducer.h>
#include <algorithm>
#include <cassert>
#include <cmath>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Image;
using namespace TMIV::Metadata;
using namespace TMIV::Renderer;

namespace TMIV::ViewOptimizer {
// TODO(BK): Expose hidden parameter
constexpr auto overlapThreshold = 0.5;
constexpr auto halfPixel = 0.5F;

ViewReducer::ViewReducer(const Json & /*unused*/,
                         const Common::Json & /*unused*/) {}

auto ViewReducer::optimizeIntraPeriod(CameraParametersList cameras)
    -> Output<CameraParametersList> {
  Output<CameraParametersList> result;
  m_priorities.assign(cameras.size(), false);

  // choose 9 degree as quantization step of angle between view i and view j.
  const float degree_step = radperdeg * 9;
  // choose 64 as quantization step of FOV.
  const float FOV_step = (45 * radperdeg) / 4;

  size_t nbCameras = cameras.size();

  // search the largest angle between two views i,j
  vector<pair<size_t, size_t>> cameras_id_pair;
  pair<size_t, size_t> camera_id_pair;

  // Decide whether number of high priority list is one
  bool isoneview = false;

  size_t id_i;
  size_t id_j;

  // Early termination: if any view is full-ERP, choose this view
  for (auto &camera : cameras) {
    if (camera.type == ProjectionType::ERP) {
      if (abs(camera.erpPhiRange[0] - camera.erpPhiRange[1]) == fullCycle) {
        if (abs(camera.erpThetaRange[0] - camera.erpThetaRange[1]) ==
            halfCycle) {
          isoneview = true;
          break;
        }
      }
    }
  }

  // Calculate the angle between view i and view j
  size_t max_angle = 0;
  if (!isoneview) {
    for (size_t id_1 = 0; id_1 < nbCameras - 1; id_1++) {
      for (size_t id_2 = id_1 + 1; id_2 < nbCameras; id_2++) {
        // Sphere distance function
        auto temp_angle =
            size_t(acos(sin(cameras[id_1].rotation[1] * radperdeg) *
                            sin(cameras[id_2].rotation[1] * radperdeg) +
                        cos(cameras[id_1].rotation[1] * radperdeg) *
                            cos(cameras[id_2].rotation[1] * radperdeg) *
                            cos((cameras[id_1].rotation[0] -
                                 cameras[id_2].rotation[0]) *
                                radperdeg)) /
                   degree_step);

        if (temp_angle > max_angle) {
          cameras_id_pair.clear();
          cameras_id_pair.emplace_back(id_1, id_2);
          max_angle = temp_angle;
        } else if (temp_angle == max_angle) {
          cameras_id_pair.emplace_back(id_1, id_2);
        }
      }
    }
    // Calculate the sum of two views' FOV
    size_t max_FOV = 0;
    size_t max_num = 0;
    for (size_t id = 0; id < cameras_id_pair.size(); id++) {
      size_t temp_FOV = 0;
      size_t id_1 = cameras_id_pair[id].first;
      size_t id_2 = cameras_id_pair[id].second;

      temp_FOV = static_cast<size_t>(
          (calculateFOV(cameras[id_1]) + calculateFOV(cameras[id_2])) /
          FOV_step);

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
    max_num = 0;
    for (size_t id = 0; id < cameras_id_pair.size(); id++) {
      float temp_distance = 0;
      size_t id_1 = cameras_id_pair[id].first;
      size_t id_2 = cameras_id_pair[id].second;

      temp_distance = calculateDistance(cameras[id_1], cameras[id_2]);
      if (temp_distance > max_distance) {
        max_distance = temp_distance;
        max_num = 1;
        swap(cameras_id_pair[0], cameras_id_pair[id]);
      } else if (temp_distance == max_distance) {
        swap(cameras_id_pair[max_num], cameras_id_pair[id]);
        max_num++;
      }
    }
    cameras_id_pair.erase(cameras_id_pair.begin() + max_num,
                          cameras_id_pair.end());

    float min_distance = numeric_limits<float>::max();

    for (auto &id : cameras_id_pair) {
      float temp_distance = 0;
      size_t id_1 = id.first;
      size_t id_2 = id.second;

      temp_distance =
          abs(cameras[id_1].position[0] - cameras[id_2].position[0]);
      if (temp_distance < min_distance) {
        min_distance = temp_distance;
        camera_id_pair = id;
      }
    }

    // Calculte the overlap of view pair
    id_i = camera_id_pair.first;
    id_j = camera_id_pair.second;
    float overlapping = 0;
    overlapping = calculateOverlapping(cameras[id_i], cameras[id_j]);

    // Decide whether the number is one or multiple
    isoneview =
        overlapping >= overlapThreshold * min(calculateFOV(cameras[id_i]),
                                              calculateFOV(cameras[id_j]));
  }

  // Just select 1 view which has the shortest distance to center
  if (isoneview) {
    float x_center = 0;
    float y_center = 0;
    float z_center = 0;
    int id_center = 0;
    float distance = numeric_limits<float>::max();

    for (auto &camera : cameras) {
      x_center += camera.position[0];
      y_center += camera.position[1];
      z_center += camera.position[2];
    }
    x_center /= nbCameras;
    y_center /= nbCameras;
    z_center /= nbCameras;

    vector<size_t> camera_id;
    size_t max_FOV = 0;
    // Search views which have the largest FOV
    for (size_t id = 0; id < nbCameras; id++) {
      size_t temp_FOV = 0;

      temp_FOV = static_cast<size_t>(calculateFOV(cameras[id]) / FOV_step);

      if (temp_FOV > max_FOV) {
        max_FOV = temp_FOV;
        camera_id.clear();
        camera_id.push_back(id);
      } else if (temp_FOV == max_FOV) {
        camera_id.push_back(id);
      }
    }
    // Search views which have the least diatance to center
    for (auto i : camera_id) {
      float temp_distance = sqrt(square(cameras[i].position[0] - x_center) +
                                 square(cameras[i].position[1] - y_center) +
                                 square(cameras[i].position[2] - z_center));
      if (temp_distance < distance) {
        id_center = int(i);
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

  // Move cameras into basic and additional partitions
  for (size_t index = 0; index != cameras.size(); ++index) {
    (m_priorities[index] ? result.basic : result.additional)
        .push_back(cameras[index]);
  }
  return result;
}

auto ViewReducer::optimizeFrame(MVD16Frame views) const -> Output<MVD16Frame> {
  Output<MVD16Frame> result;
  assert(m_priorities.size() == views.size());

  // Move views into basic and additional partitions
  for (size_t index = 0; index != views.size(); ++index) {
    (m_priorities[index] ? result.basic : result.additional)
        .push_back(move(views[index]));
  }
  return result;
}

auto ViewReducer::calculateFOV(CameraParameters camera) -> float {
  float temp_FOV = 0;

  if (camera.type == ProjectionType::ERP) {
    temp_FOV = abs(camera.erpPhiRange[0] - camera.erpPhiRange[1]) * radperdeg *
               (abs(sin(camera.erpThetaRange[0] * radperdeg) -
                    sin(camera.erpThetaRange[1] * radperdeg)));
  } else if (camera.type == ProjectionType::Perspective) {
    temp_FOV =
        abs(4 * atan(camera.size[0] / (2 * camera.perspectiveFocal[0])) *
            sin(atan(camera.size[1] / (2 * camera.perspectiveFocal[1]))));
  }
  return temp_FOV;
}
auto ViewReducer::calculateDistance(CameraParameters camera_1,
                                    CameraParameters camera_2) -> float {
  return sqrt(square(camera_1.position[0] - camera_2.position[0]) +
              square(camera_1.position[1] - camera_2.position[1]) +
              square(camera_1.position[2] - camera_2.position[2]));
}
auto ViewReducer::calculateOverlapping(Metadata::CameraParameters camera_from,
                                       Metadata::CameraParameters camera_to)
    -> float {

  float overlapping = 0.0F;
  float weight_all = 0.0F;
  float weight_overlapped = 0.0F;

  Mat<Vec2f> gridMapToProject = imagePositions(camera_from);
  Mat<float> depth;
  depth.resize(camera_from.size.y(), camera_from.size.x());

  Mat<int> isoverlap;
  isoverlap.resize(camera_from.size.y(), camera_from.size.x());
  constexpr auto bitDepth = 16;
  float depth_temp = sqrtf(expandDepthValue<bitDepth>(camera_from, 1) *
                           expandDepthValue<bitDepth>(camera_from, UINT16_MAX));

  for (unsigned i = 0; i != depth.height(); ++i) {
    for (unsigned j = 0; j != depth.width(); ++j) {
      isoverlap(i, j) = 0;
      depth(i, j) = depth_temp;
    }
  }
  auto ptsOncamera_to =
      reprojectPoints(camera_from, camera_to, gridMapToProject, depth);

  int lastXPruned = camera_to.size.x() - 1;
  int lastYPruned = camera_to.size.y() - 1;

  for (unsigned i = 0; i != depth.height(); ++i) {
    for (unsigned j = 0; j != depth.width(); ++j) {
      const Vec2f &position = ptsOncamera_to.first[i * depth.width() + j];

      if (lround(position.x()) >= 0 && lround(position.x()) < lastXPruned &&
          lround(position.y()) >= 0 && lround(position.y()) < lastYPruned) {
        isoverlap(i, j) = 1;
      }
    }
  }

  for (unsigned i = 0; i != isoverlap.height(); ++i) {
    for (unsigned j = 0; j != isoverlap.width(); ++j) {
      float weight = 0.0F;
      if (camera_from.type == ProjectionType::ERP) {
        float angle =
            (camera_from.erpThetaRange[1] +
             (float(i) + halfPixel) *
                 (camera_from.erpThetaRange[0] - camera_from.erpThetaRange[1]) /
                 isoverlap.height()) *
            radperdeg;
        // calculate weight of each pixel in sphere
        weight = cos(angle);
      } else if (camera_from.type == ProjectionType::Perspective) {
        weight = 1;
      }
      weight_all += weight;
      if (isoverlap(i, j) != 0) {
        weight_overlapped += weight;
      }
    }
  }
  overlapping = weight_overlapped / weight_all * calculateFOV(camera_from);
  return overlapping;
}
} // namespace TMIV::ViewOptimizer
