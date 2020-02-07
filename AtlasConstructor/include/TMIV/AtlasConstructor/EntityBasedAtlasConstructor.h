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

#ifndef _TMIV_ATLASCONSTRUCTOR_ENTITYBASEDATLASCONSTRUCTOR_H_
#define _TMIV_ATLASCONSTRUCTOR_ENTITYBASEDATLASCONSTRUCTOR_H_

#include <TMIV/AtlasConstructor/IAtlasConstructor.h>

#include <TMIV/AtlasConstructor/IAggregator.h>
#include <TMIV/AtlasConstructor/IPacker.h>
#include <TMIV/AtlasConstructor/IPruner.h>
#include <TMIV/Common/Json.h>

#include <deque>
#include <memory>

namespace TMIV::AtlasConstructor {
class EntityBasedAtlasConstructor : public IAtlasConstructor {
public:
  EntityBasedAtlasConstructor(const Common::Json & /*rootNode*/,
                              const Common::Json & /*componentNode*/);
  EntityBasedAtlasConstructor(const EntityBasedAtlasConstructor &) = delete;
  EntityBasedAtlasConstructor(EntityBasedAtlasConstructor &&) = default;
  EntityBasedAtlasConstructor &operator=(const EntityBasedAtlasConstructor &) = delete;
  EntityBasedAtlasConstructor &operator=(EntityBasedAtlasConstructor &&) = default;
  ~EntityBasedAtlasConstructor() override = default;

  auto prepareSequence(Metadata::IvSequenceParams ivSequenceParams, std::vector<bool> isBasicView)
      -> const Metadata::IvSequenceParams & override;
  void prepareAccessUnit(Metadata::IvAccessUnitParams ivAccessUnitParams) override;
  void pushFrame(Common::MVD16Frame transportViews) override;
  auto completeAccessUnit() -> const Metadata::IvAccessUnitParams & override;
  auto popAtlas() -> Common::MVD16Frame override;

private:
  static Common::MVD16Frame entitySeparator(Common::MVD16Frame transportViews,
                                            Common::EntityMapList entityMaps, uint16_t entityId);
  static std::vector<Common::Frame<Common::YUV420P16>> yuvSampler(const Common::EntityMapList &in);
  static void mergeViews(Common::MVD16Frame &entityMergedViews,
                         Common::MVD16Frame transportEntityViews);
  static void mergeMasks(Common::MaskList &entityMergedMasks, Common::MaskList masks);
  static void updateMasks(const Common::MVD16Frame &views, Common::MaskList &masks);
  void updateEntityMasks(Common::EntityMapList &entityMasks, const Common::MaskList &masks,
                         uint16_t entityId);
  void aggregateEntityMasks(Common::MaskList &Masks, std::uint16_t entityId);
  void swap0(Common::EntityMapList &entityMasks);
  static auto setView(const Common::TextureDepth16Frame &view, const Common::EntityMap &entityMask,
                      int entityId) -> Common::TextureDepth16Frame;
  void writePatchInAtlas(const Metadata::AtlasParameters &patch, const Common::MVD16Frame &views,
                         Common::MVD16Frame &atlas);

  std::size_t m_nbAtlas{};
  Common::Vec2i m_atlasSize;
  Common::Vec2i m_EntityEncRange;
  Common::Json m_rootNode;
  std::unique_ptr<IPruner> m_pruner;
  std::unique_ptr<IAggregator> m_aggregator;
  std::unique_ptr<IPacker> m_packer;
  std::vector<bool> m_isBasicView;
  std::vector<Common::MVD16Frame> m_viewBuffer;
  Metadata::IvSequenceParams m_ivSequenceParams;
  Metadata::IvAccessUnitParams m_ivAccessUnitParams;
  std::deque<Common::MVD16Frame> m_atlasBuffer;
  int m_fIndex{0};
  std::vector<Common::MaskList> m_aggregatedEntityMask;
  std::vector<Common::EntityMapList> m_entityMasksBuffer;
  unsigned m_maxEntities{};
  int m_frameInGOPIndex{0};
};
} // namespace TMIV::AtlasConstructor

#endif
