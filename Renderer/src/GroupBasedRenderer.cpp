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

#include <TMIV/Renderer/GroupBasedRenderer.h>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/Factory.h>

#include <cmath>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::Renderer {
GroupBasedRenderer::GroupBasedRenderer(const Json &rootNode, const Json &componentNode) {
  m_synthesizer =
      Factory<ISynthesizer>::getInstance().create("Synthesizer", rootNode, componentNode);
  m_inpainter = Factory<IInpainter>::getInstance().create("Inpainter", rootNode, componentNode);
}

namespace {
template <class InIt1, class InIt2, class InIt3, class InIt4, class OutIt, class Fn>
void my_transform(InIt1 i1, InIt1 end1, InIt2 i2, InIt3 i3, InIt4 i4, OutIt dest, Fn Func) {
  for (; i1 != end1; ++i1, ++i2, ++i3, ++i4, ++dest) {
    *dest = Func(*i1, *i2, *i3, *i4);
  }
}

enum class MergeMode {
  inpaint = 0, // let the inpainter fill
  lowPass = 1, // fill from the low-pass synthesis results which are in the background
  highPass = 2 // fill from the high-pass synthesis results which are in the foreground
};

struct GroupBasedRendererHelper {
  MergeMode mergeMode{};
  vector<size_t> numberOfViewsPerPass;
  vector<unsigned> selectedGViewsPass;
  vector<unsigned> patchesGViewId;

  auto filterGMergeDepth(uint16_t i, uint16_t j) const -> uint16_t {
    if (i > 0) {
      if (i >= j) { // Checking depth
        return i;
      }
      // Handle conflict
      switch (mergeMode) {
      case MergeMode::inpaint:
        return 0;
      case MergeMode::lowPass:
        return i;
      case MergeMode::highPass:
        return j;
      default:
        abort();
      }
    }
    return j;
  }

  auto filterGMergeTexture(uint16_t i, uint16_t j, uint16_t id, uint16_t jd) const -> uint16_t {
    if (i > 0) {
      if (id >= jd) { // Checking depth
        return i;
      }
      // Handle conflict
      switch (mergeMode) {
      case MergeMode::inpaint:
        return 0;
      case MergeMode::lowPass:
        return i;
      case MergeMode::highPass:
        return j;
      default:
        abort();
      }
    }
    return j;
  }

  auto filterGMaps(uint16_t i) const -> uint16_t {
    if (i != unusedPatchId && contains(selectedGViewsPass, patchesGViewId[i])) {
      return i;
    }
    return unusedPatchId;
  }

  // NOTE: Mutates numberOfViewsPerPass
  auto sortGViews(const vector<vector<uint16_t>> &viewsPerGroup, const ViewParamsVector &cameras,
                  const ViewParams &target) /*mutable*/ -> vector<size_t> {
    vector<vector<size_t>> sortedGCamerasId;
    vector<float> distanceG;
    size_t camSum = 0;

    for (const auto &views : viewsPerGroup) {
      vector<float> distance;
      vector<float> angle;
      vector<float> angleWeight;

      for (size_t i = 0; i < views.size(); i++) {
        const auto &source = cameras[views[i]];
        distance.push_back(norm(source.position - target.position));

        // Compute Angle between the camera and target in degree unit
        const auto yaw_target = target.rotation[0] * radperdeg;
        const auto yaw_source = source.rotation[0] * radperdeg;
        const auto pitch_target = target.rotation[1] * radperdeg;
        const auto pitch_source = source.rotation[1] * radperdeg;
        angle.push_back(degperrad *
                        acos(sin(pitch_source) * sin(pitch_target) +
                             cos(pitch_source) * cos(pitch_target) * cos(yaw_source - yaw_target)));

        // to assure angle is ranging from -180 to 180 degree
        if (angle[i] > halfCycle) {
          angle[i] -= fullCycle;
        }

        // Introduce AngleWeight as a simple triangle function (with value of 1 when
        // angle is 0 & value of 0 when angle is 180)
        angleWeight.push_back(1.F - abs(angle[i]) / halfCycle);
      }

      // Find the sorted cameras indices per group
      vector<size_t> sortedCamerasId(views.size());
      iota(sortedCamerasId.begin(), sortedCamerasId.end(), 0);
      sort(sortedCamerasId.begin(), sortedCamerasId.end(),
           [&distance, &angleWeight](size_t i1, size_t i2) {
             if (angleWeight[i1] == angleWeight[i2]) {
               return distance[i1] < distance[i2];
             }
             return distance[i1] * (1.0 - angleWeight[i1]) < distance[i2] * (1.0 - angleWeight[i2]);
           });
      distanceG.push_back(distance[sortedCamerasId[0]]);
      for (auto &viewId : sortedCamerasId) {
        viewId += camSum;
      }
      sortedGCamerasId.push_back(sortedCamerasId);
      camSum += sortedCamerasId.size();
    }

    // Find the nearest group
    vector<int> sortedDistanceId(distanceG.size());
    iota(sortedDistanceId.begin(), sortedDistanceId.end(), 0);
    sort(sortedDistanceId.begin(), sortedDistanceId.end(),
         [&distanceG](size_t i1, size_t i2) { return distanceG[i1] < distanceG[i2]; });

    vector<size_t> sortedGCamerasIdFinal;
    camSum = 0;
    for (auto dIndex : sortedDistanceId) {
      const auto &views = sortedGCamerasId[dIndex];
      sortedGCamerasIdFinal.insert(sortedGCamerasIdFinal.end(), begin(views), end(views));
      camSum += views.size();
      numberOfViewsPerPass.push_back(camSum);
    }
    return sortedGCamerasIdFinal;
  }
};
} // namespace

auto GroupBasedRenderer::renderFrame(const MVD10Frame &atlas, const PatchIdMapList &maps,
                                     const IvSequenceParams &ivSequenceParams,
                                     const IvAccessUnitParams &ivAccessUnitParams,
                                     const ViewParams &target) const -> Texture444Depth16Frame {
  GroupBasedRendererHelper helper;

  assert(ivAccessUnitParams.atlasParamsList);
  const auto &atlasParamsList = *ivAccessUnitParams.atlasParamsList;

  // Initialization
  helper.mergeMode =
      ivSequenceParams.depthLowQualityFlag ? MergeMode::lowPass : MergeMode::highPass;

  auto groupIds = vector<unsigned>(atlas.size(), 0);
  if (atlasParamsList.groupIds) {
    groupIds = *atlasParamsList.groupIds;
  }

  const int numberOfPasses = ivSequenceParams.numGroups;

  // initalize mapsPass by unusedPatchId
  vector<PatchIdMapList> mapsPass(numberOfPasses);
  for (auto &pass : mapsPass) {
    for (const auto &patchIds : maps) {
      PatchIdMap patchMap(patchIds.getWidth(), patchIds.getHeight());
      fill(patchMap.getPlane(0).begin(), patchMap.getPlane(0).end(), unusedPatchId);
      pass.push_back(patchMap);
    }
  }

  for (const auto &patch : atlasParamsList) {
    helper.patchesGViewId.push_back(patch.viewId);
  }

  // Find views per group
  auto viewsPerGroup = vector<vector<uint16_t>>(ivSequenceParams.numGroups);
  for (const auto &patch : atlasParamsList) {
    auto &viewIds = viewsPerGroup[groupIds[patch.atlasId]];
    if (!contains(viewIds, patch.viewId)) {
      viewIds.push_back(patch.viewId);
    }
  }

  // Ordering views based on their distance & angle to target view
  const auto sortedCamerasId =
      helper.sortGViews(viewsPerGroup, ivSequenceParams.viewParamsList, target);
  if (helper.numberOfViewsPerPass.size() == 0) { // in case of numGroups == 1
    helper.numberOfViewsPerPass.push_back(ivSequenceParams.viewParamsList.size());
  }

  // Produce the individual pass synthesis results
  auto viewportPass = vector<Texture444Depth16Frame>(numberOfPasses);
  for (int passId = 0; passId < numberOfPasses; passId++) {
    // Find the selected views for a given pass
    helper.selectedGViewsPass.clear();
    for (size_t id = 0; id < ivSequenceParams.viewParamsList.size(); ++id) {
      if (id < helper.numberOfViewsPerPass[passId]) {
        helper.selectedGViewsPass.push_back(static_cast<unsigned int>(sortedCamerasId[id]));
      }
    }

	cout << "Selected Optimized Views in Pass " << passId << " : ";
    for (size_t i = 0; i < helper.selectedGViewsPass.size(); i++) {
      cout << "o" << helper.selectedGViewsPass[i] << ", ";
    }
    cout << '\n';

    // Update the Occupancy Map to be used in the Pass
    for (size_t atlasId = 0; atlasId < maps.size(); ++atlasId) {
      transform(maps[atlasId].getPlane(0).begin(), maps[atlasId].getPlane(0).end(),
                mapsPass[passId][atlasId].getPlane(0).begin(),
                [&helper](auto i) { return helper.filterGMaps(i); });
    }

    // Synthesis per pass
    viewportPass[passId] = m_synthesizer->renderFrame(atlas, mapsPass[passId], ivSequenceParams,
                                                      ivAccessUnitParams, target);
  }

  // Merging
  auto viewport = viewportPass[numberOfPasses - 1];
  for (auto passId = numberOfPasses - 1; passId > 0; passId--) {
    transform(viewportPass[passId - 1].second.getPlane(0).begin(),
              viewportPass[passId - 1].second.getPlane(0).end(),
              viewport.second.getPlane(0).begin(), viewport.second.getPlane(0).begin(),
              [&helper](auto i, auto j) { return helper.filterGMergeDepth(i, j); });

    for (auto i = 0; i < viewport.first.getNumberOfPlanes(); ++i) {
      my_transform(
          viewportPass[passId - 1].first.getPlane(i).begin(),
          viewportPass[passId - 1].first.getPlane(i).end(), viewport.first.getPlane(i).begin(),
          viewportPass[passId - 1].second.getPlane(0).begin(), viewport.second.getPlane(0).begin(),
          viewport.first.getPlane(i).begin(), [&helper](auto i, auto j, auto id, auto jd) {
            return helper.filterGMergeTexture(i, j, id, jd);
          });
    }
  }
  m_inpainter->inplaceInpaint(viewport, target);
  return viewport;
}
} // namespace TMIV::Renderer
