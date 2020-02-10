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

#ifndef _TMIV_RENDERER_GROUPBASEDRENDERER_H_
#define _TMIV_RENDERER_GROUPBASEDRENDERER_H_

#include <TMIV/Renderer/IInpainter.h>
#include <TMIV/Renderer/IRenderer.h>
#include <TMIV/Renderer/ISynthesizer.h>
#include <TMIV/Renderer/IViewingSpaceController.h>

#include <bitset>

namespace TMIV::Renderer {
// Advanced GroupBased implementation of IRenderer
class GroupBasedRenderer : public IRenderer {
private:
  std::unique_ptr<ISynthesizer> m_synthesizer;
  std::unique_ptr<IInpainter> m_inpainter;
  std::unique_ptr<IViewingSpaceController> m_viewingSpaceController;

public:
  GroupBasedRenderer(const Common::Json & /*rootNode*/, const Common::Json & /*componentNode*/);
  GroupBasedRenderer(const GroupBasedRenderer &) = delete;
  GroupBasedRenderer(GroupBasedRenderer &&) = default;
  GroupBasedRenderer &operator=(const GroupBasedRenderer &) = delete;
  GroupBasedRenderer &operator=(GroupBasedRenderer &&) = default;
  ~GroupBasedRenderer() override = default;

  auto renderFrame(const Common::MVD10Frame &atlases, const Common::PatchIdMapList &patchIdMapList,
                   const MivBitstream::IvSequenceParams &ivSequenceParams,
                   const MivBitstream::IvAccessUnitParams &ivAccessUnitParams,
                   const MivBitstream::ViewParams &target) const
      -> Common::Texture444Depth16Frame override;

private:
  using GroupIdMask = std::bitset<32>;

  // Render multiple groups
  auto renderPass(GroupIdMask groupIdMask, const Common::MVD10Frame &atlases,
                  const Common::PatchIdMapList &patchIdMapList,
                  const MivBitstream::IvSequenceParams &ivSequenceParams,
                  const MivBitstream::IvAccessUnitParams &ivAccessUnitParams,
                  const MivBitstream::ViewParams &target) const -> Common::Texture444Depth16Frame;

  // Determine group render order (multipass rendering)
  static auto groupRenderOrder(const MivBitstream::IvSequenceParams &ivSequenceParams,
                               const MivBitstream::IvAccessUnitParams &ivAccessUnitParams,
                               const MivBitstream::ViewParams &target) -> std::vector<unsigned>;

  // Filter the patch id map list for groups included in the mask
  static auto filterPatchIdMapList(GroupIdMask groupIdMask, Common::PatchIdMapList patchIdMapList,
                                   const MivBitstream::IvAccessUnitParams &ivAccessUnitParams)
      -> Common::PatchIdMapList;

  struct Priority {
    float distance;
    float angleWeight;

    auto operator<(const Priority &other) const -> bool;
  };

  // Determine the priority of a group
  static auto groupPriority(unsigned groupId, const MivBitstream::IvSequenceParams &ivSequenceParams,
                            const MivBitstream::IvAccessUnitParams &ivAccessUnitParams,
                            const MivBitstream::ViewParams &target) -> Priority;

  // Determine the priority of a view
  static auto viewPriority(const MivBitstream::ViewParams &source, const MivBitstream::ViewParams &target)
      -> Priority;

  enum class MergeMode {
    inpaint = 0,   // let the inpainter fill
    lowPass = 1,   // fill always from the lower pass whether belong to foreground or background
    foreground = 2 // fill always from the foreground whether belong to lower pass or high pass
  };

  // Merge a render pass into the partial render result
  static void inplaceMerge(Common::Texture444Depth16Frame &viewport,
                           const Common::Texture444Depth16Frame &viewportPass, MergeMode mergeMode);
};
} // namespace TMIV::Renderer

#endif
