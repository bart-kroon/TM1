/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
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
using namespace TMIV::MivBitstream;

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
}

namespace {
struct MultipassRendererHelper {
  vector<unsigned> selectedViewsPass;
  vector<unsigned> patchesViewId;

  [[nodiscard]] auto filterMaps(uint16_t i) const -> uint16_t {
    if (i != unusedPatchId && contains(selectedViewsPass, patchesViewId[i])) {
      return i;
    }
    return unusedPatchId;
  }

  static auto sortViews(const ViewParamsList &viewParamsList, const ViewParams &target)
      -> vector<size_t> {
    vector<float> distance;
    vector<float> angle;
    vector<float> angleWeight;

    for (size_t i = 0; i < viewParamsList.size(); ++i) {
      const auto &source = viewParamsList[i];
      distance.push_back(norm(source.ce.position() - target.ce.position()));

      // Compute angle between the camera and target
      angle.push_back(greatCircleDistance(source.ce.rotation(), target.ce.rotation()));

      // Introduce AngleWeight as a simple triangle function (with value of 1 when
      // angle is 0 & value of 0 when angle is 180)
      angleWeight.push_back(1.F - angle[i] / halfCycle);
    }

    // Find the sorted viewParamsList indices
    vector<size_t> sortedCamerasId(viewParamsList.size());
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

auto MultipassRenderer::renderFrame(const AccessUnit &frame, const ViewParams &viewportParams) const
    -> Texture444Depth16Frame {
  MultipassRendererHelper helper;
  const auto &viewParamsList = frame.atlas.front().viewParamsList;

  // Filter out all patches across all atlases
  auto framePass = frame;
  for (auto &atlas : framePass.atlas) {
    fill(atlas.blockToPatchMap.getPlane(0).begin(), atlas.blockToPatchMap.getPlane(0).end(),
         unusedPatchId);

    // Precondition: all atlases use the same view parameters list
    if (atlas.viewParamsList != viewParamsList) {
      throw runtime_error(
          "The MultipassRenderer requires that all atlases share the same view parameters list");
    }

    for (const auto &patch : atlas.patchParamsList) {
      helper.patchesViewId.push_back(patch.pduViewId());
    }
  }

  // Ordering views based on their distance & angle to target view
  const auto sortedCamerasId = helper.sortViews(viewParamsList, viewportParams);

  // Produce the individual pass synthesis results
  auto viewportPass = vector<Texture444Depth16Frame>(m_numberOfPasses);
  for (size_t passId = 0; passId < m_numberOfPasses; passId++) {
    // Find the selected views for a given pass
    helper.selectedViewsPass.clear();
    for (size_t i = 0; i < viewParamsList.size(); ++i) {
      if (i < m_numberOfViewsPerPass[passId]) {
        helper.selectedViewsPass.push_back(unsigned(sortedCamerasId[i]));
      }
    }

    cout << "Selected Optimized Views in Pass " << passId << " : ";
    for (auto i : helper.selectedViewsPass) {
      cout << "o" << i << ", ";
    }
    cout << "\n";

    // Update the block to patch map to be used in this pass
    for (size_t atlasId = 0; atlasId < frame.atlas.size(); ++atlasId) {
      transform(frame.atlas[atlasId].blockToPatchMap.getPlane(0).begin(),
                frame.atlas[atlasId].blockToPatchMap.getPlane(0).end(),
                framePass.atlas[atlasId].blockToPatchMap.getPlane(0).begin(),
                [&helper](auto i) { return helper.filterMaps(i); });
    }

    // Synthesis per pass
    viewportPass[passId] = m_synthesizer->renderFrame(framePass, viewportParams);
  }

  // Merging
  auto viewport = viewportPass[m_numberOfPasses - 1];
  for (auto passId = m_numberOfPasses - 1; passId > 0; passId--) {
    for (size_t i = 0; i < viewport.first.getPlane(0).size(); i++) {
      if (viewportPass[passId - 1].second.getPlane(0)[i] != 0) {
        // Always copy from the lower pass synthesis results if there is content there.
        viewport.second.getPlane(0)[i] = viewportPass[passId - 1].second.getPlane(0)[i];
        for (int planeId = 0; planeId < viewport.first.getNumberOfPlanes(); planeId++) {
          viewport.first.getPlane(planeId)[i] = viewportPass[passId - 1].first.getPlane(planeId)[i];
        }
      }
    }
  }

  if (frame.vps->miv_sequence_params().msp_max_entities_minus1() == 0) {
    m_inpainter->inplaceInpaint(viewport, viewportParams);
  }

  if (frame.vs) {
    m_viewingSpaceController->inplaceFading(viewport, viewportParams, *frame.vs);
  }

  return viewport;
}
} // namespace TMIV::Renderer
