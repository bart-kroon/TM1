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
using namespace TMIV::MivBitstream;

namespace TMIV::Renderer {
GroupBasedRenderer::GroupBasedRenderer(const Json &rootNode, const Json &componentNode) {
  m_synthesizer =
      Factory<ISynthesizer>::getInstance().create("Synthesizer", rootNode, componentNode);
  m_inpainter = Factory<IInpainter>::getInstance().create("Inpainter", rootNode, componentNode);
  m_viewingSpaceController = Factory<IViewingSpaceController>::getInstance().create(
      "ViewingSpaceController", rootNode, componentNode);
}

auto GroupBasedRenderer::renderFrame(const AccessUnit &frame,
                                     const ViewParams &viewportParams) const
    -> Texture444Depth16Frame {
  const auto &msp = frame.vps->miv_sequence_params();
  if (msp.msp_num_groups_minus1() >= GroupIdMask{}.size()) {
    throw runtime_error("This decoder implementation is limited to a maximum number of groups");
  }

  // Determine group render order
  const auto groupIdPass = groupRenderOrder(frame, viewportParams);

  // Render all passes
  auto viewportPass = vector<Texture444Depth16Frame>(msp.msp_num_groups_minus1() + 1);
  auto groupIdMask = GroupIdMask{};
  for (size_t pass = 0; pass <= msp.msp_num_groups_minus1(); ++pass) {
    groupIdMask.set(groupIdPass[pass]);
    viewportPass[pass] = renderPass(groupIdMask, frame, viewportParams);
  }

  // Merge passes
  auto viewport = move(viewportPass.back());
  for (auto pass = msp.msp_num_groups_minus1(); pass > 0; --pass) {
    inplaceMerge(viewport, viewportPass[pass - 1],
                 msp.msp_depth_low_quality_flag() ? MergeMode::lowPass : MergeMode::foreground);
  }

  // Inpainting
  if (msp.msp_max_entities_minus1() == 0) {
    m_inpainter->inplaceInpaint(viewport, viewportParams);
  }

  // Viewing space handling
  if (frame.vs) {
    m_viewingSpaceController->inplaceFading(viewport, viewportParams, *frame.vs);
  }

  return viewport;
}

auto GroupBasedRenderer::groupRenderOrder(const AccessUnit &frame, const ViewParams &viewportParams)
    -> std::vector<unsigned> {
  auto &msp = frame.vps->miv_sequence_params();
  auto groupPriorities = vector<Priority>();
  auto result = vector<unsigned>();

  // Build array of group priorities
  for (unsigned groupId = 0; groupId <= msp.msp_num_groups_minus1(); ++groupId) {
    groupPriorities.push_back(groupPriority(groupId, frame, viewportParams));
    result.push_back(groupId);
  }

  // Sort by priority
  sort(begin(result), end(result),
       [&](unsigned i, unsigned j) { return groupPriorities[i] < groupPriorities[j]; });

  // Return sort key
  return result;
}

auto GroupBasedRenderer::renderPass(GroupIdMask groupIdMask, const AccessUnit &frame,
                                    const ViewParams &viewportParams) const
    -> Texture444Depth16Frame {
  return m_synthesizer->renderFrame(filterFrame(groupIdMask, frame), viewportParams);
}

auto GroupBasedRenderer::filterFrame(GroupIdMask groupIdMask, AccessUnit frame)
    -> MivBitstream::AccessUnit {
  for (auto &atlas : frame.atlas) {
    const auto &masp = atlas.asps.miv_atlas_sequence_params();
    if (!groupIdMask.test(masp.masp_group_id())) {
      fill(atlas.blockToPatchMap.getPlane(0).begin(), atlas.blockToPatchMap.getPlane(0).end(),
           unusedPatchId);
    }
  }
  return frame;
}

auto GroupBasedRenderer::groupPriority(unsigned groupId, const AccessUnit &frame,
                                       const ViewParams &viewportParams) -> Priority {
  auto result = optional<Priority>{};

  // For each atlas in this group
  for (auto &atlas : frame.atlas) {
    if (groupId == atlas.asps.miv_atlas_sequence_params().masp_group_id()) {
      // Once for each referenced view
      vector<bool> once(atlas.viewParamsList.size(), false);
      for (auto &patch : atlas.patchParamsList) {
        if (!once[patch.pduViewId()]) {
          once[patch.pduViewId()] = true;
          const auto &viewParams = atlas.viewParamsList[patch.pduViewId()];

          // Find the view with the highest priority (i.e. closest to the target view)
          const auto priority = viewPriority(viewParams, viewportParams);
          if (!result || priority < *result) {
            result = priority;
          }
        }
      }
    }
  }

  return result ? *result : Priority{};
}

auto GroupBasedRenderer::viewPriority(const ViewParams &view1, const ViewParams &view2)
    -> Priority {
  const auto distance = norm(view1.ce.position() - view2.ce.position());

  // Compute angle between the camera and target
  const auto v1 = rotate(Vec3f{1.F, 0.F, 0.F}, view1.ce.rotation());
  const auto v2 = rotate(Vec3f{1.F, 0.F, 0.F}, view2.ce.rotation());
  const auto angle = acos(dot(v1, v2));

  // Introduce angleWeight as a simple triangle function (with value of 1 when
  // angle is 0 & value of 0 when angle is 180)
  const auto angleWeight = 1.F - angle / halfCycle;

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
