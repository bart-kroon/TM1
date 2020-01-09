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

#include <TMIV/Renderer/MultipassRenderer.h>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/Factory.h>

#include <cmath>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::Renderer {
MultipassRenderer::MultipassRenderer(const Json &rootNode, const Json &componentNode) {
  m_synthesizer =
      Factory<ISynthesizer>::getInstance().create("Synthesizer", rootNode, componentNode);
  m_inpainter = Factory<IInpainter>::getInstance().create("Inpainter", rootNode, componentNode);
  m_viewingSpaceController = Factory<IViewingSpaceController>::getInstance().create(
      "ViewingSpaceController", rootNode, componentNode);
  m_numberOfPasses = size_t(componentNode.require("NumberOfPasses").asInt());
  const auto subnode = componentNode.require("NumberOfViewsPerPass");
  for (size_t i = 0; i != subnode.size(); ++i) {
    m_numberOfViewsPerPass.push_back(size_t(subnode.at(i).asInt()));
  }
  if (m_numberOfPasses != m_numberOfViewsPerPass.size()) {
    throw runtime_error("NumberOfPasses and NumberOfViewsPerPass are inconsistent");
  }
  if (auto subnode = componentNode.optional("MergeConflict")) {
    m_mergeConflict = MergeMode(subnode.asInt());
  }
}

namespace {
template <class InIt1, class InIt2, class InIt3, class InIt4, class OutIt, class Fn>
void my_transform(InIt1 i1, InIt1 end1, InIt2 i2, InIt3 i3, InIt4 i4, OutIt dest, Fn Func) {
  for (; i1 != end1; ++i1, ++i2, ++i3, ++i4, ++dest) {
    *dest = Func(*i1, *i2, *i3, *i4);
  }
}

struct MultipassRendererHelper {
  MergeMode mergeConflict{};
  vector<unsigned> selectedViewsPass;
  vector<unsigned> patchesViewId;

  auto filterMergeTexture(uint16_t i, uint16_t j, uint16_t id, uint16_t jd) const -> uint16_t {
    if (i > 0) {
      if (id >= jd) { // Checking depth
        return i;
      }
      // Handle conflict
      switch (mergeConflict) {
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

  auto filterMergeDepth(uint16_t i, uint16_t j) const -> uint16_t {
    return filterMergeTexture(i, j, i, j);
  }

  auto filterMaps(uint16_t i) const -> uint16_t {
    if (i != unusedPatchId && contains(selectedViewsPass, patchesViewId[i])) {
      return i;
    }
    return unusedPatchId;
  }

  static auto sortViews(const ViewParamsVector &viewParamsVector, const ViewParams &target)
      -> vector<size_t> {
    vector<float> distance;
    vector<float> angle;
    vector<float> angleWeight;

    for (size_t i = 0; i < viewParamsVector.size(); ++i) {
      const auto &source = viewParamsVector[i];
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

    // Find the sorted viewParamsVector indices
    vector<size_t> sortedCamerasId(viewParamsVector.size());
    iota(sortedCamerasId.begin(), sortedCamerasId.end(), 0);
    sort(sortedCamerasId.begin(), sortedCamerasId.end(),
         [&distance, &angleWeight](size_t i1, size_t i2) {
           if (angleWeight[i1] == angleWeight[i2]) {
             return distance[i1] < distance[i2];
           }
           return distance[i1] * (1.F - angleWeight[i1]) < distance[i2] * (1.F - angleWeight[i2]);
         });
    return sortedCamerasId;
  }
};
} // namespace

auto MultipassRenderer::renderFrame(const MVD10Frame &atlas, const PatchIdMapList &maps,
                                    const IvSequenceParams &ivSequenceParams,
                                    const IvAccessUnitParams &ivAccessUnitParams,
                                    const ViewParams &target) const -> Texture444Depth16Frame {
  MultipassRendererHelper helper;

  helper.mergeConflict = m_mergeConflict;

  assert(ivAccessUnitParams.atlasParamsList);
  const auto &atlasParamsList = *ivAccessUnitParams.atlasParamsList;

  // Initalize mapsPass by unusedPatchId
  auto mapsPass = vector<PatchIdMapList>(m_numberOfPasses);
  for (auto &pass : mapsPass) {
    for (const auto &patchIds : maps) {
      PatchIdMap patchMap(patchIds.getWidth(), patchIds.getHeight());
      fill(patchMap.getPlane(0).begin(), patchMap.getPlane(0).end(), unusedPatchId);
      pass.push_back(patchMap);
    }
  }

  for (const auto &patch : atlasParamsList) {
    helper.patchesViewId.push_back(patch.viewId);
  }

  // Ordering views based on their distance & angle to target view
  const auto sortedCamerasId = helper.sortViews(ivSequenceParams.viewParamsList, target);

  // Produce the individual pass synthesis results
  auto viewportPass = vector<Texture444Depth16Frame>(m_numberOfPasses);
  for (size_t passId = 0; passId < m_numberOfPasses; passId++) {
    // Find the selected views for a given pass
    helper.selectedViewsPass.clear();
    for (size_t i = 0; i < ivSequenceParams.viewParamsList.size(); ++i) {
      if (i < m_numberOfViewsPerPass[passId]) {
        helper.selectedViewsPass.push_back(unsigned(sortedCamerasId[i]));
      }
    }

    cout << "Selected Optimized Views in Pass " << passId << " : ";
    for (auto i : helper.selectedViewsPass) {
      cout << "o" << i << ", ";
    }
    cout << "\n";

    // Update the Occupancy Map to be used in the Pass
    for (size_t atlasId = 0; atlasId < maps.size(); ++atlasId) {
      transform(maps[atlasId].getPlane(0).begin(), maps[atlasId].getPlane(0).end(),
                mapsPass[passId][atlasId].getPlane(0).begin(),
                [&helper](auto i) { return helper.filterMaps(i); });
    }

    // Synthesis per pass
    viewportPass[passId] = m_synthesizer->renderFrame(atlas, mapsPass[passId], ivSequenceParams,
                                                      ivAccessUnitParams, target);
  }

  // Merging
  auto viewport = viewportPass[m_numberOfPasses - 1];
  for (auto passId = m_numberOfPasses - 1; passId > 0; passId--) {
    transform(viewportPass[passId - 1].second.getPlane(0).begin(),
              viewportPass[passId - 1].second.getPlane(0).end(),
              viewport.second.getPlane(0).begin(), viewport.second.getPlane(0).begin(),
              [&helper](auto i, auto j) { return helper.filterMergeDepth(i, j); });

    for (auto i = 0; i < viewport.first.getNumberOfPlanes(); ++i) {
      my_transform(
          viewportPass[passId - 1].first.getPlane(i).begin(),
          viewportPass[passId - 1].first.getPlane(i).end(), viewport.first.getPlane(i).begin(),
          viewportPass[passId - 1].second.getPlane(0).begin(), viewport.second.getPlane(0).begin(),
          viewport.first.getPlane(i).begin(), [&helper](auto i, auto j, auto id, auto jd) {
            return helper.filterMergeTexture(i, j, id, jd);
          });
    }
  }

  m_inpainter->inplaceInpaint(viewport, target);

  // fading to grey with respect to viewing space
  if (ivSequenceParams.viewingSpace)
    m_viewingSpaceController->inplaceFading(viewport, target, ivSequenceParams);

  return viewport;
}
} // namespace TMIV::Renderer
