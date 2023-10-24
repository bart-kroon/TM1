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
#include "FramePacker.h"
#include "SampleStats.h"

#include <TMIV/Aggregator/IAggregator.h>
#include <TMIV/Encoder/Encoder.h>
#include <TMIV/Packer/IPacker.h>
#include <TMIV/Pruner/IPruner.h>
#include <TMIV/ViewOptimizer/IViewOptimizer.h>

#include <algorithm>
#include <memory>

namespace TMIV::Encoder {
using TextureStats = Common::stack::Vec3<SampleStats>;
using PatchTextureStats = std::vector<TextureStats>;

class Encoder::Impl {
public:
  explicit Impl(const Common::Json &componentNode);

  void prepareSequence(const MivBitstream::SourceUnit &unit);
  void prepareAccessUnit();
  void pushFrame(Common::DeepFrameList sourceViews);
  auto completeAccessUnit() -> const EncoderParams &;
  auto popAtlas() -> Common::V3cFrameList;
  [[nodiscard]] auto maxLumaSamplesPerFrame() const -> size_t;

private:
  [[nodiscard]] auto config() const noexcept -> const Configuration & { return m_config; }

  // Encoder_pushFrame.cpp
  void pushSingleEntityFrame(Common::DeepFrameList sourceViews);
  void pushMultiEntityFrame(Common::DeepFrameList sourceViews);
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
  void constructVideoFrames();
  void filterPatchMargins();
  void clearPatchMargins(size_t f, size_t a, Common::Frame<> &tmpTex, Common::Frame<> &tmpGeo,
                         Common::Frame<> &tmpTmp);
  void inpaintPatchMargins(size_t f, size_t a, Common::Frame<> &tmpTex, Common::Frame<> &tmpGeo,
                           Common::Frame<> &tmpTmp);
  void blurPatchMargins(size_t f, size_t a, Common::Frame<> &tmpTex, Common::Frame<> &tmpGeo,
                        Common::Frame<> &tmpTmp);
  void encodePatchTextureOffset(const PatchTextureStats &stats);
  void applyPatchTextureOffset();
  [[nodiscard]] auto calculateBtpm() const -> std::vector<std::vector<std::vector<int32_t>>>;
  void adaptBtpmToPatchCount(std::vector<std::vector<std::vector<int32_t>>> &btpm) const;
  [[nodiscard]] auto isRedundantBlock(Common::Vec2i topLeft, Common::Vec2i bottomRight,
                                      uint16_t viewIdx) const -> bool;
  auto writePatchInAtlas(const MivBitstream::PatchParams &patchParams,
                         const Common::DeepFrame &view, Common::DeepFrameList &frame,
                         size_t patchIdx) -> TextureStats;
  void adaptAtlas(const MivBitstream::PatchParams &patchParams, Common::DeepFrame &atlas,
                  int32_t yOcc, int32_t xOcc, const Common::Vec2i &pView,
                  const Common::Vec2i &pAtlas) const;

  void setTiles();
  auto setPartition() -> bool;
  static void assignPatchesToTiles(EncoderParams &params);
  void setAtlasFrameTileInformation(bool uniformPartitionSpacingFlag);
  void repeatAtlasTileHeaders(size_t atlasIdx);
  std::vector<std::vector<std::vector<int32_t>>> m_partitionArray;

  auto plsMakeHistogram(int32_t piece_num, size_t numOfFrames, size_t v, int32_t minDepthVal,
                        int32_t maxDepthVal) -> std::vector<int32_t>;
  auto plsGeometryDynamicRange(size_t numOfFrames, size_t v, int32_t minDepthMapValWithinGOP,
                               int32_t maxDepthMapValWithinGOP, bool lowDepthQuality)
      -> std::vector<double>;

  // Encoder sub-components
  std::unique_ptr<ViewOptimizer::IViewOptimizer> m_viewOptimizer;
  std::unique_ptr<Pruner::IPruner> m_pruner;
  std::unique_ptr<Aggregator::IAggregator> m_aggregator;
  std::unique_ptr<Packer::IPacker> m_packer;
  FramePacker m_framePacker;

  Configuration m_config;

  // View-optimized encoder input
  ViewOptimizer::ViewOptimizerParams m_transportParams;
  std::vector<Common::DeepFrameList> m_transportViews;

  int32_t m_blockSize{};
  EncoderParams m_params;          // Encoder output prior to geometry quantization and scaling
  EncoderParams m_paramsQuantized; // Encoder output prior to geometry scaling
  std::vector<Common::DeepFrameList> m_videoFrameBuffer;

  // Mark read-only access to encoder params to make mutable access more visible
  [[nodiscard]] auto params() const noexcept -> const EncoderParams & { return m_params; }

  // Mask aggregation state
  std::vector<Common::FrameList<uint8_t>> m_aggregatedEntityMask;
  size_t m_maxLumaSamplesPerFrame{};

  std::vector<Common::Vec3i> m_patchColorCorrectionOffset;
};
} // namespace TMIV::Encoder

#endif
