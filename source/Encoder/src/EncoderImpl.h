/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#ifndef TMIV_ENCODER_ENCODER_IMPL_H
#define TMIV_ENCODER_ENCODER_IMPL_H

#include "Configuration.h"
#include "SampleStats.h"

#include <TMIV/Aggregator/IAggregator.h>
#include <TMIV/Encoder/Encoder.h>
#include <TMIV/Packer/IPacker.h>
#include <TMIV/Pruner/IPruner.h>

#include <algorithm>
#include <memory>

namespace TMIV::Encoder {
using TextureStats = Common::stack::Vec3<SampleStats>;
using PatchTextureStats = std::vector<TextureStats>;
using MivBitstream::EncoderParams;

class Encoder::Impl {
public:
  explicit Impl(const Common::Json &componentNode);

  [[nodiscard]] auto isStart(const SourceUnit &unit) -> bool;
  void process(std::vector<SourceUnit> buffer, const Common::StageSource<CodableUnit> &source_);

  [[nodiscard]] auto maxLumaSamplesPerFrame() const -> size_t;

private:
  void prepareSequence(const SourceUnit &unit);
  void prepareAccessUnit();
  void pushFrame(Common::DeepFrameList transportViews);
  void completeAccessUnit();

  [[nodiscard]] auto config() const noexcept -> const Configuration & { return m_config; }

  // Encoder_pushFrame.cpp
  void pushSingleEntityFrame(Common::DeepFrameList transportViews);
  void pushMultiEntityFrame(Common::DeepFrameList transportViews);
  static auto entitySeparator(const Common::DeepFrameList &transportViews,
                              Common::SampleValue entityId) -> Common::DeepFrameList;
  static auto yuvSampler(const Common::FrameList<> &in) -> Common::FrameList<>;
  static void mergeMasks(Common::FrameList<uint8_t> &mergedMasks, Common::FrameList<uint8_t> masks);
  static void updateMasks(const Common::DeepFrameList &views, Common::FrameList<uint8_t> &masks);
  void aggregateEntityMasks(Common::FrameList<uint8_t> &masks, Common::SampleValue entityId);

  // Encoder_completeAccessUnit.cpp
  void scaleChromaDynamicRange();
  void pruningWithInformation(Common::FrameList<uint8_t> &aggregatedMask,
                              const Common::FrameList<uint32_t> &information);
  void updateAggregationStatistics(const Common::FrameList<uint8_t> &aggregatedMask);
  auto allocateAtlasList(uint32_t texBitDepth) const -> Common::DeepFrameList;
  void constructVideoFrames();
  void encodePatchTextureOffset(const PatchTextureStats &stats);
  void applyPatchTextureOffset();
  [[nodiscard]] auto calculateBtpm() const -> std::vector<std::vector<std::vector<int32_t>>>;
  void adaptBtpmToPatchCount(std::vector<std::vector<std::vector<int32_t>>> &btpm) const;
  [[nodiscard]] auto isRedundantBlock(Common::Vec2i topLeft, Common::Vec2i bottomRight,
                                      uint16_t viewIdx) const -> bool;

  template <typename Invocable>
  void visitPatch(const MivBitstream::PatchParams &patchParams, Invocable &&visit) const;

  auto writeSampleInPatchIdxMap(const MivBitstream::PatchParams &patchParams,
                                Common::DeepFrameList &frame, size_t patchIdx) const;
  auto writeOccupancySampleInAtlas(const MivBitstream::PatchParams &patchParams,
                                   const Common::DeepFrame &view,
                                   Common::DeepFrameList &frame) const;
  auto writeGeometrySampleInAtlas(const MivBitstream::PatchParams &patchParams,
                                  const Common::DeepFrame &view,
                                  Common::DeepFrameList &frame) const;
  auto writeTextureSampleInAtlas(const MivBitstream::PatchParams &patchParams,
                                 const Common::DeepFrame &view, Common::DeepFrameList &frame) const;

  void writePatchInAtlas(const MivBitstream::PatchParams &patchParams,
                         const Common::DeepFrame &view, Common::DeepFrameList &frame,
                         size_t patchIdx);
  auto collectPatchTextureStats(const MivBitstream::PatchParams &patchParams,
                                const Common::DeepFrameList &frame) -> TextureStats;

  void setTiles();
  auto setPartition() -> bool;
  void setAtlasFrameTileInformation(bool uniformPartitionSpacingFlag);

  std::vector<std::vector<std::vector<int32_t>>> m_partitionArray;

  // Encoder sub-components
  std::unique_ptr<Pruner::IPruner> m_pruner;
  std::unique_ptr<Aggregator::IAggregator> m_aggregator;
  std::unique_ptr<Packer::IPacker> m_packer;

  Configuration m_config;

  // View-optimized encoder input
  MivBitstream::ViewParamsList m_transportViewParams;
  int32_t m_semiBasicViewCount{};
  std::vector<Common::DeepFrameList> m_transportViews;

  int32_t m_blockSize{};
  EncoderParams m_params;
  int32_t m_firstIdx{};
  int32_t m_lastIdx{};

  std::vector<Common::DeepFrameList> m_videoFrameBuffer;

  // Mark read-only access to encoder params to make mutable access more visible
  [[nodiscard]] auto params() const noexcept -> const EncoderParams & { return m_params; }

  // Mask aggregation state
  std::vector<Common::FrameList<uint8_t>> m_aggregatedEntityMask;
  size_t m_maxLumaSamplesPerFrame{};
};
} // namespace TMIV::Encoder

#endif
