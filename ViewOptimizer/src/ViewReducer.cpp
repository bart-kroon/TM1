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

#include <TMIV/ViewOptimizer/ViewReducer.h>

#include <TMIV/Common/Common.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <TMIV/Renderer/reprojectPoints.h>

#include <algorithm>
#include <cassert>
#include <cmath>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;
using namespace TMIV::Renderer;

namespace TMIV::ViewOptimizer {
// TODO(BK): Expose hidden parameter
constexpr auto overlapThreshold = 0.5;
constexpr auto halfPixel = 0.5F;

ViewReducer::ViewReducer(const Json & /*unused*/, const Json & /*unused*/) {}

auto ViewReducer::optimizeSequence(IvSequenceParams ivSequenceParams) -> Output {

  const auto &viewParamsVector = ivSequenceParams.viewParamsList;
  m_isBasicView.assign(viewParamsVector.size(), false);

  // choose 9 degree as quantization step of angle between view i and view j.
  const float degree_step = radperdeg * 9;
  // choose 64 as quantization step of FOV.
  const float FOV_step = (45 * radperdeg) / 4;

  size_t nbCameras = viewParamsVector.size();

  // search the largest angle between two views i,j
  vector<pair<size_t, size_t>> cameras_id_pair;
  pair<size_t, size_t> camera_id_pair;

  // Decide whether number of high priority list is one
  bool isoneview = false;

  size_t id_i;
  size_t id_j;

  // Early termination: if any view is full-ERP, choose this view
  for (auto &viewParams : viewParamsVector) {
    if (auto projection = get_if<ErpParams>(&viewParams.projection)) {
      if (abs(projection->phiRange[0] - projection->phiRange[1]) == fullCycle) {
        if (abs(projection->thetaRange[0] - projection->thetaRange[1]) == halfCycle) {
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
        // TODO(BK): Reimplement angle comparison with quaternions
#if false
        auto temp_angle = size_t(
            acos(sin(viewParamsVector[id_1].rotation[1] * radperdeg) *
                     sin(viewParamsVector[id_2].rotation[1] * radperdeg) +
                 cos(viewParamsVector[id_1].rotation[1] * radperdeg) *
                     cos(viewParamsVector[id_2].rotation[1] * radperdeg) *
                     cos((viewParamsVector[id_1].rotation[0] - viewParamsVector[id_2].rotation[0]) *
                         radperdeg)) /
            degree_step);
#else
        auto temp_angle = 0;
#endif

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
          (calculateFOV(viewParamsVector[id_1]) + calculateFOV(viewParamsVector[id_2])) / FOV_step);

      if (temp_FOV > max_FOV) {
        max_FOV = temp_FOV;
        max_num = 1;
        swap(cameras_id_pair[0], cameras_id_pair[id]);
      } else if (temp_FOV == max_FOV) {
        swap(cameras_id_pair[max_num], cameras_id_pair[id]);
        max_num++;
      }
    }
    cameras_id_pair.erase(cameras_id_pair.begin() + max_num, cameras_id_pair.end());

    // Select a pair of view which is farest to each other
    float max_distance = -1;
    max_num = 0;
    for (size_t id = 0; id < cameras_id_pair.size(); id++) {
      float temp_distance = 0;
      size_t id_1 = cameras_id_pair[id].first;
      size_t id_2 = cameras_id_pair[id].second;

      temp_distance = calculateDistance(viewParamsVector[id_1], viewParamsVector[id_2]);
      if (temp_distance > max_distance) {
        max_distance = temp_distance;
        max_num = 1;
        swap(cameras_id_pair[0], cameras_id_pair[id]);
      } else if (temp_distance == max_distance) {
        swap(cameras_id_pair[max_num], cameras_id_pair[id]);
        max_num++;
      }
    }
    cameras_id_pair.erase(cameras_id_pair.begin() + max_num, cameras_id_pair.end());

    float min_distance = numeric_limits<float>::max();

    for (auto &id : cameras_id_pair) {
      float temp_distance = 0;
      size_t id_1 = id.first;
      size_t id_2 = id.second;

      // TODO(BK): This must be a mistake. Why only the x-coordinate?
      temp_distance = abs(viewParamsVector[id_1].ce.ce_view_pos_x() -
                          viewParamsVector[id_2].ce.ce_view_pos_x());
      if (temp_distance < min_distance) {
        min_distance = temp_distance;
        camera_id_pair = id;
      }
    }

    // Calculte the overlap of view pair
    id_i = camera_id_pair.first;
    id_j = camera_id_pair.second;
    float overlapping = 0;
    overlapping = calculateOverlapping(viewParamsVector[id_i], viewParamsVector[id_j]);

    // Decide whether the number is one or multiple
    isoneview = overlapping >= overlapThreshold * min(calculateFOV(viewParamsVector[id_i]),
                                                      calculateFOV(viewParamsVector[id_j]));
  }

  // Just select 1 view which has the shortest distance to center
  if (isoneview) {
    float x_center = 0;
    float y_center = 0;
    float z_center = 0;
    int id_center = 0;
    float distance = numeric_limits<float>::max();

    for (auto &viewParams : viewParamsVector) {
      x_center += viewParams.ce.ce_view_pos_x();
      y_center += viewParams.ce.ce_view_pos_y();
      z_center += viewParams.ce.ce_view_pos_z();
    }
    x_center /= nbCameras;
    y_center /= nbCameras;
    z_center /= nbCameras;

    vector<size_t> camera_id;
    size_t max_FOV = 0;
    // Search views which have the largest FOV
    for (size_t id = 0; id < nbCameras; id++) {
      size_t temp_FOV = 0;

      temp_FOV = static_cast<size_t>(calculateFOV(viewParamsVector[id]) / FOV_step);

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
      float temp_distance = sqrt(square(viewParamsVector[i].ce.ce_view_pos_x() - x_center) +
                                 square(viewParamsVector[i].ce.ce_view_pos_y() - y_center) +
                                 square(viewParamsVector[i].ce.ce_view_pos_z() - z_center));
      if (temp_distance < distance) {
        id_center = int(i);
        distance = temp_distance;
      }
    }
    m_isBasicView[id_center] = true;
  }
  // Just select 2 view i and j
  else {
    m_isBasicView[camera_id_pair.first] = true;
    m_isBasicView[camera_id_pair.second] = true;
  }

  // Output
  return {std::move(ivSequenceParams), m_isBasicView};
}

auto ViewReducer::calculateFOV(ViewParams viewParams) -> float {
  return visit(overload(
                   [](const ErpParams &projection) {
                     return abs(projection.phiRange[0] - projection.phiRange[1]) * radperdeg *
                            (abs(sin(projection.thetaRange[0] * radperdeg) -
                                 sin(projection.thetaRange[1] * radperdeg)));
                   },
                   [&](const PerspectiveParams &projection) {
                     return abs(
                         4 * atan(viewParams.projectionPlaneSize[0] / (2 * projection.focal[0])) *
                         sin(atan(viewParams.projectionPlaneSize[1] / (2 * projection.focal[1]))));
                   }),
               viewParams.projection);
}
auto ViewReducer::calculateDistance(ViewParams camera_1, ViewParams camera_2) -> float {
  return norm(camera_1.ce.position() - camera_2.ce.position());
}
auto ViewReducer::calculateOverlapping(ViewParams camera_from, ViewParams camera_to) -> float {

  float overlapping = 0.0F;
  float weight_all = 0.0F;
  float weight_overlapped = 0.0F;

  Mat<Vec2f> gridMapToProject = imagePositions(camera_from);
  Mat<float> depth;
  depth.resize(camera_from.projectionPlaneSize.y(), camera_from.projectionPlaneSize.x());

  Mat<int> isoverlap;
  isoverlap.resize(camera_from.projectionPlaneSize.y(), camera_from.projectionPlaneSize.x());
  const auto depthTransform = DepthTransform<16>{camera_from.dq};
  const float middleDepth =
      sqrtf(depthTransform.expandDepth(1) * depthTransform.expandDepth(UINT16_MAX));

  for (unsigned i = 0; i != depth.height(); ++i) {
    for (unsigned j = 0; j != depth.width(); ++j) {
      isoverlap(i, j) = 0;
      depth(i, j) = middleDepth;
    }
  }
  auto ptsOncamera_to = reprojectPoints(camera_from, camera_to, gridMapToProject, depth);

  int lastXPruned = camera_to.projectionPlaneSize.x() - 1;
  int lastYPruned = camera_to.projectionPlaneSize.y() - 1;

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
      const float weight =
          visit(overload(
                    [&](const ErpParams &projection) { // calculate weight of each pixel in sphere
                      float angle = (projection.thetaRange[1] +
                                     (float(i) + halfPixel) *
                                         (projection.thetaRange[0] - projection.thetaRange[1]) /
                                         isoverlap.height()) *
                                    radperdeg;
                      return cos(angle);
                    },
                    [](const PerspectiveParams & /* unused */) { return 1.F; }),
                camera_from.projection);
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
