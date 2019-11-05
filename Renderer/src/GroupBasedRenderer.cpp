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
                 ivSequenceParams.depthLowQualityFlag ? MergeMode::lowPass : MergeMode::highPass);
  }

  // Inpainting
  m_inpainter->inplaceInpaint(viewport, target);
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
  const auto & groupIds = *ivAccessUnitParams.atlasParamsList->groupIds;
  for (const auto &patch : *ivAccessUnitParams.atlasParamsList) {
    if (groupId == groupIds[patch.atlasId] && !contains(viewIds, patch.viewId)) {
      viewIds.push_back(patch.viewId);
    }
  }

  // Do something in case there are no patches
  if (viewIds.empty()) {
    return {};
  }

  // Find the view with the highest priority (i.e. the view within the group that is closest to the target view)
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

namespace {
template <class InIt1, class InIt2, class InIt3, class InIt4, class OutIt, class Fn>
void transform4(InIt1 i1, InIt1 end1, InIt2 i2, InIt3 i3, InIt4 i4, OutIt dest, Fn Func) {
  for (; i1 != end1; ++i1, ++i2, ++i3, ++i4, ++dest) {
    *dest = Func(*i1, *i2, *i3, *i4);
  }
}
} // namespace

void GroupBasedRenderer::inplaceMerge(Texture444Depth16Frame &viewport,
                                      const Texture444Depth16Frame &viewportPass,
                                      MergeMode mergeMode) {
  transform(viewportPass.second.getPlane(0).begin(), // i's
            viewportPass.second.getPlane(0).end(),   //
            viewport.second.getPlane(0).begin(),     // j's
            viewport.second.getPlane(0).begin(),     // result
            [=](auto i, auto j) { return filterMergeDepth(i, j, mergeMode); });

  for (int d = 0; d < viewportPass.first.getNumberOfPlanes(); ++d) {
    transform4(viewportPass.first.getPlane(d).begin(),  // i's
               viewportPass.first.getPlane(d).end(),    //
               viewport.first.getPlane(d).begin(),      // j's
               viewportPass.second.getPlane(0).begin(), // id's
               viewport.second.getPlane(0).begin(),     // jd's
               viewport.first.getPlane(d).begin(),      // result
               [=](auto i, auto j, auto id, auto jd) {
                 return filterMergeTexture(i, j, id, jd, mergeMode);
               });
  }
}

auto GroupBasedRenderer::filterMergeDepth(uint16_t i, uint16_t j, MergeMode mergeMode) -> uint16_t {
  return filterMergeTexture(i, j, i, j, mergeMode);
}

auto GroupBasedRenderer::filterMergeTexture(uint16_t i, uint16_t j, uint16_t id, uint16_t jd,
                                            MergeMode mergeMode) -> uint16_t {
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

auto GroupBasedRenderer::Priority::operator<(const Priority &other) const -> bool {
  if (angleWeight == other.angleWeight)
    return distance < other.distance;
  else
	return distance * (1.F - angleWeight) < other.distance * (1.F - other.angleWeight);
}

} // namespace TMIV::Renderer
