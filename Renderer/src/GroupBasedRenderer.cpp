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
  m_viewingSpaceController = Factory<IViewingSpaceController>::getInstance().create(
      "ViewingSpaceController", rootNode, componentNode);
}

auto GroupBasedRenderer::renderFrame(const MVD10Frame &atlases,
                                     const PatchIdMapList &patchIdMapList,
                                     const IvSequenceParams &ivSequenceParams,
                                     const IvAccessUnitParams &ivAccessUnitParams,
                                     const ViewParams &target) const -> Texture444Depth16Frame {
  if (ivSequenceParams.numGroups >= GroupIdMask{}.size()) {
    throw runtime_error("This decoder implementation is limited to a maximum number of groups");
  }

  // Determine group render order
  const auto groupIdPass = groupRenderOrder(ivSequenceParams, ivAccessUnitParams, target);

  // Render all passes
  auto viewportPass = vector<Texture444Depth16Frame>(ivSequenceParams.numGroups);
  auto groupIdMask = GroupIdMask{};
  for (size_t pass = 0; pass < ivSequenceParams.numGroups; ++pass) {
    groupIdMask.set(groupIdPass[pass]);
    viewportPass[pass] = renderPass(groupIdMask, atlases, patchIdMapList, ivSequenceParams,
                                    ivAccessUnitParams, target);
  }

  // Merge passes
  auto viewport = move(viewportPass.back());
  for (auto pass = ivSequenceParams.numGroups - 1; pass > 0; --pass) {
    inplaceMerge(viewport, viewportPass[pass - 1],
                 ivSequenceParams.depthLowQualityFlag ? MergeMode::lowPass : MergeMode::foreground);
  }

  // Inpainting
    m_inpainter->inplaceInpaint(viewport, target);

  // fading to grey with respect to viewing space
  if (ivSequenceParams.viewingSpace) {
    m_viewingSpaceController->inplaceFading(viewport, target, ivSequenceParams);
  }

  return viewport;
}

auto GroupBasedRenderer::groupRenderOrder(const Metadata::IvSequenceParams &ivSequenceParams,
                                          const Metadata::IvAccessUnitParams &ivAccessUnitParams,
                                          const Metadata::ViewParams &target)
    -> std::vector<unsigned> {
  auto groupPriorities = vector<Priority>();
  auto result = vector<unsigned>();

  // Build array of group priorities
  for (unsigned groupId = 0; groupId < ivSequenceParams.numGroups; ++groupId) {
    groupPriorities.push_back(groupPriority(groupId, ivSequenceParams, ivAccessUnitParams, target));
    result.push_back(groupId);
  }

  // Sort by priority
  sort(begin(result), end(result),
       [&](unsigned i, unsigned j) { return groupPriorities[i] < groupPriorities[j]; });

  // Return sort key
  return result;
}

auto GroupBasedRenderer::renderPass(GroupIdMask groupIdMask, const MVD10Frame &atlases,
                                    const PatchIdMapList &patchIdMapList,
                                    const IvSequenceParams &ivSequenceParams,
                                    const IvAccessUnitParams &ivAccessUnitParams,
                                    const ViewParams &target) const -> Texture444Depth16Frame {
  return m_synthesizer->renderFrame(
      atlases, filterPatchIdMapList(groupIdMask, patchIdMapList, ivAccessUnitParams),
      ivSequenceParams, ivAccessUnitParams, target);
}

auto GroupBasedRenderer::filterPatchIdMapList(GroupIdMask groupIdMask,
                                              PatchIdMapList patchIdMapList,
                                              const IvAccessUnitParams &ivAccessUnitParams)
    -> PatchIdMapList {
  assert(ivAccessUnitParams.atlasParamsList);

  // No grouping, no filtering
  if (!ivAccessUnitParams.atlasParamsList->groupIds) {
    assert(groupIdMask == 1);
    return patchIdMapList;
  }

  // Filter out atlases that belong to a group that is not selected for this pass
  const auto &groupIds = *ivAccessUnitParams.atlasParamsList->groupIds;
  for (size_t atlasId = 0; atlasId < patchIdMapList.size(); ++atlasId) {
    if (!groupIdMask.test(groupIds[atlasId])) {
      fill(patchIdMapList[atlasId].getPlane(0).begin(), patchIdMapList[atlasId].getPlane(0).end(),
           unusedPatchId);
    }
  }

  return patchIdMapList;
}

auto GroupBasedRenderer::groupPriority(unsigned groupId, const IvSequenceParams &ivSequenceParams,
                                       const IvAccessUnitParams &ivAccessUnitParams,
                                       const ViewParams &target) -> Priority {
  // No grouping, no priority
  assert(ivAccessUnitParams.atlasParamsList);
  if (!ivAccessUnitParams.atlasParamsList->groupIds) {
    assert(groupId == 0);
    return {};
  }

  // Enumerate the views that occur in this group (in arbitrary order)
  vector<unsigned> viewIds;
  viewIds.reserve(ivSequenceParams.viewParamsList.size());
  const auto &groupIds = *ivAccessUnitParams.atlasParamsList->groupIds;
  for (const auto &patch : *ivAccessUnitParams.atlasParamsList) {
    if (groupId == groupIds[patch.atlasId] && !contains(viewIds, patch.viewId)) {
      viewIds.push_back(patch.viewId);
    }
  }

  // Do something in case there are no patches
  if (viewIds.empty()) {
    return {};
  }

  // Find the view with the highest priority (i.e. the view within the group that is closest to the
  // target view)
  const auto highest = *min_element(begin(viewIds), end(viewIds), [&](unsigned i, unsigned j) {
    return viewPriority(ivSequenceParams.viewParamsList[i], target) <
           viewPriority(ivSequenceParams.viewParamsList[j], target);
  });

  // Return that priority
  return viewPriority(ivSequenceParams.viewParamsList[highest], target);
}

auto GroupBasedRenderer::viewPriority(const ViewParams &source, const ViewParams &target)
    -> Priority {
  const auto distance = norm(source.position - target.position);

  // Compute angle between the camera and target in degree unit
  const auto yaw_target = target.rotation[0] * radperdeg;
  const auto yaw_source = source.rotation[0] * radperdeg;
  const auto pitch_target = target.rotation[1] * radperdeg;
  const auto pitch_source = source.rotation[1] * radperdeg;
  auto angle =
      degperrad * acos(sin(pitch_source) * sin(pitch_target) +
                       cos(pitch_source) * cos(pitch_target) * cos(yaw_source - yaw_target));

  // to assure angle is ranging from -180 to 180 degree
  if (angle > halfCycle) {
    angle -= fullCycle;
  }

  // Introduce angleWeight as a simple triangle function (with value of 1 when
  // angle is 0 & value of 0 when angle is 180)
  const auto angleWeight = (1.F - abs(angle) / halfCycle);

  return {distance, angleWeight};
}

void GroupBasedRenderer::inplaceMerge(Texture444Depth16Frame &viewport,
                                      const Texture444Depth16Frame &viewportPass,
                                      MergeMode mergeMode) {
  for (size_t i = 0; i < viewport.first.getPlane(0).size(); i++) {
    if (viewportPass.second.getPlane(0)[i] != 0) {
      if (viewport.second.getPlane(0)[i] <= viewportPass.second.getPlane(0)[i]) {
        // copy from lower pass synthesis results which have content from foreground objects
        viewport.second.getPlane(0)[i] = viewportPass.second.getPlane(0)[i];
        for (int planeId = 0; planeId < viewport.first.getNumberOfPlanes(); planeId++) {
          viewport.first.getPlane(planeId)[i] = viewportPass.first.getPlane(planeId)[i];
        }
      } else {
        // Handle conflict
        switch (mergeMode) {
        case MergeMode::inpaint:
          // put 0 in depth map, neutral color in texture, and let inpainter handle it
          viewport.second.getPlane(0)[i] = 0;
          for (int planeId = 0; planeId < viewport.first.getNumberOfPlanes(); planeId++) {
            viewport.first.getPlane(planeId)[i] = TextureFrame::neutralColor();
          }
          break;
        case MergeMode::lowPass:
          // Always copy from the lower pass synthesis results if there is content there
          viewport.second.getPlane(0)[i] = viewportPass.second.getPlane(0)[i];
          for (int planeId = 0; planeId < viewport.first.getNumberOfPlanes(); planeId++) {
            viewport.first.getPlane(planeId)[i] = viewportPass.first.getPlane(planeId)[i];
          }
          break;
        case MergeMode::foreground:
          break; // do nothing, as foreground objects will be always copyied from when merging.
        default:
          abort();
        }
      }
    }
  }
}

auto GroupBasedRenderer::Priority::operator<(const Priority &other) const -> bool {
  // avoid 0 < 0 when angleWeight == other.angleWeight == 1
  if (angleWeight == other.angleWeight) {
    return distance < other.distance;
  }
  return distance * (1.F - angleWeight) < other.distance * (1.F - other.angleWeight);
}
} // namespace TMIV::Renderer
