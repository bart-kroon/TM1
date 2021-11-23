/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#ifndef TMIV_ENCODER_ENCODER_H
#define TMIV_ENCODER_ENCODER_H

#include <TMIV/Aggregator/IAggregator.h>
#include <TMIV/DepthQualityAssessor/IDepthQualityAssessor.h>
#include <TMIV/Encoder/Configuration.h>
#include <TMIV/Encoder/FramePacker.h>
#include <TMIV/MivBitstream/SequenceConfig.h>
#include <TMIV/Packer/IPacker.h>
#include <TMIV/Pruner/IPruner.h>
#include <TMIV/ViewOptimizer/IViewOptimizer.h>

#include <bitset>
#include <deque>
#include <memory>

namespace TMIV::Encoder {
auto assessColorConsistency(Common::DeepFrameList views, MivBitstream::ViewParamsList params)
    -> std::vector<Common::Mat<Common::Vec3i>>;

class Encoder {
public:
  Encoder(const Common::Json &rootNode, const Common::Json &componentNode);

  void prepareSequence(const MivBitstream::SequenceConfig &sequenceConfig,
                       const Common::DeepFrameList &firstFrame);
  void prepareAccessUnit();
  void pushFrame(Common::DeepFrameList sourceViews);
  auto completeAccessUnit() -> const EncoderParams &;
  auto popAtlas() -> Common::V3cFrameList;
  [[nodiscard]] auto maxLumaSamplesPerFrame() const -> size_t;

  [[nodiscard]] auto config() const noexcept -> const Configuration & { return m_config; }

private:
  // Encoder_prepareAccessUnit.cpp
  void resetNonAggregatedMask();

  // Encoder_pushFrame.cpp
  void pushSingleEntityFrame(Common::DeepFrameList sourceViews);
  void updateNonAggregatedMask(const Common::DeepFrameList &transportViews,
                               const Common::FrameList<uint8_t> &masks);
  void pushMultiEntityFrame(Common::DeepFrameList sourceViews);
  static auto entitySeparator(const Common::DeepFrameList &transportViews,
                              Common::SampleValue entityId) -> Common::DeepFrameList;
  static auto yuvSampler(const Common::FrameList<> &in) -> Common::FrameList<>;
  static void mergeMasks(Common::FrameList<uint8_t> &mergedMasks, Common::FrameList<uint8_t> masks);
  static void updateMasks(const Common::DeepFrameList &views, Common::FrameList<uint8_t> &masks);
  void aggregateEntityMasks(Common::FrameList<uint8_t> &masks, Common::SampleValue entityId);

  // Encoder_completeAccessUnit.cpp
  void scaleGeometryDynamicRange();
  void updateAggregationStatistics(const Common::FrameList<uint8_t> &aggregatedMask);
  void constructVideoFrames();
  void correctColors();
  void calculateAttributeOffset(
      std::vector<std::array<std::array<int64_t, 4>, 3>> patchAttrOffsetValuesFullGOP);
  auto calculatePatchAttrOffsetValuesFullGOP(
      std::vector<std::array<std::array<int64_t, 4>, 3>> &patchAttrOffsetValuesFullGOP) -> int32_t;
  [[nodiscard]] auto calculateBtpm() const -> std::vector<std::vector<std::vector<int32_t>>>;
  void adaptBtpmToPatchCount(std::vector<std::vector<std::vector<int32_t>>> &btpm) const;
  [[nodiscard]] auto isRedundantBlock(Common::Vec2i topLeft, Common::Vec2i bottomRight,
                                      uint16_t viewIdx, int32_t frameIdx) const -> bool;
  auto writePatchInAtlas(const MivBitstream::PatchParams &patchParams,
                         const Common::DeepFrame &view, Common::DeepFrameList &frame,
                         int32_t frameIdx, size_t patchIdx)
      -> std::array<std::array<int64_t, 4>, 3>;
  void adaptAtlas(const MivBitstream::PatchParams &patchParams, Common::DeepFrame &atlas,
                  int32_t yOcc, int32_t xOcc, const Common::Vec2i &pView,
                  const Common::Vec2i &pAtlas) const;

  // Encoder_popFrame.cpp
  void incrementFoc();

  // Encoder sub-components
  std::unique_ptr<DepthQualityAssessor::IDepthQualityAssessor> m_depthQualityAssessor;
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
  std::deque<Common::DeepFrameList> m_videoFrameBuffer;

  // Mark read-only access to encoder params to make mutable access more visible
  [[nodiscard]] auto params() const noexcept -> const EncoderParams & { return m_params; }

  // Mask aggregation state
  using NonAggregatedMask = Common::Mat<std::bitset<maxIntraPeriod>>;
  std::vector<NonAggregatedMask> m_nonAggregatedMask;
  std::vector<Common::FrameList<uint8_t>> m_aggregatedEntityMask;
  size_t m_maxLumaSamplesPerFrame{};

  std::vector<std::vector<Common::Mat<Common::Vec3i>>> m_colorCorrectionMaps;
  std::vector<Common::Vec3i> m_patchColorCorrectionOffset;
};
} // namespace TMIV::Encoder

#endif
