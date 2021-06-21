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
#include <TMIV/Encoder/GeometryDownscaler.h>
#include <TMIV/Encoder/GeometryQuantizer.h>
#include <TMIV/MivBitstream/SequenceConfig.h>
#include <TMIV/Packer/IPacker.h>
#include <TMIV/Pruner/IPruner.h>
#include <TMIV/ViewOptimizer/IViewOptimizer.h>

#include <bitset>
#include <deque>
#include <memory>

namespace TMIV::Encoder {
auto assessColorConsistency(Common::MVD16Frame views, MivBitstream::ViewParamsList params)
    -> std::vector<Common::Mat<Common::Vec3i>>;

class Encoder {
public:
  Encoder(const Common::Json &rootNode, const Common::Json &componentNode);

  void prepareSequence(const MivBitstream::SequenceConfig &sequenceConfig,
                       const Common::MVD16Frame &firstFrame);
  void prepareAccessUnit();
  void pushFrame(Common::MVD16Frame sourceViews);
  auto completeAccessUnit() -> const EncoderParams &;
  auto popAtlas() -> Common::MVD10Frame;
  [[nodiscard]] auto maxLumaSamplesPerFrame() const -> size_t;

private: // Encoder_prepareSequence.cpp
  [[nodiscard]] auto
  calculateNominalAtlasFrameSizes(const MivBitstream::ViewParamsList &viewParamsList,
                                  double frameRate) const -> Common::SizeVector;
  [[nodiscard]] auto calculateViewGridSize(const MivBitstream::ViewParamsList &viewParamsList) const
      -> Common::Vec2i;
  [[nodiscard]] auto createVps(const std::vector<Common::Vec2i> &atlasFrameSizes) const
      -> MivBitstream::V3cParameterSet;
  [[nodiscard]] auto vuiParameters() const -> MivBitstream::VuiParameters;
  void setGiGeometry3dCoordinatesBitdepthMinus1();
  void enableOccupancyPerView();
  void prepareIvau();
  [[nodiscard]] auto log2FocLsbMinus4() const -> uint8_t;
  [[nodiscard]] auto patchSizeQuantizers() const -> Common::Vec2i;

  // Encoder_prepareAccessUnit.cpp
  void resetNonAggregatedMask();

  // Encoder_pushFrame.cpp
  void pushSingleEntityFrame(Common::MVD16Frame sourceViews);
  void updateNonAggregatedMask(const Common::MVD16Frame &transportViews,
                               const Common::MaskList &masks);
  void pushMultiEntityFrame(Common::MVD16Frame sourceViews);
  static auto entitySeparator(const Common::MVD16Frame &transportViews,
                              Common::SampleValue entityId) -> Common::MVD16Frame;
  static auto yuvSampler(const Common::EntityMapList &in)
      -> std::vector<Common::Frame<Common::YUV420P16>>;
  static void mergeMasks(Common::MaskList &mergedMasks, Common::MaskList masks);
  static void updateMasks(const Common::MVD16Frame &views, Common::MaskList &masks);
  void aggregateEntityMasks(Common::MaskList &masks, Common::SampleValue entityId);

  // Encoder_completeAccessUnit.cpp
  void scaleGeometryDynamicRange();
  void updateAggregationStatistics(const Common::MaskList &aggregatedMask);
  void constructVideoFrames();
  void correctColors();
  void calculateAttributeOffset(
      std::vector<std::array<std::array<int64_t, 4>, 3>> patchAttrOffsetValuesFullGOP);
  auto calculatePatchAttrOffsetValuesFullGOP(
      std::vector<std::array<std::array<int64_t, 4>, 3>> &patchAttrOffsetValuesFullGOP) -> int;
  [[nodiscard]] auto calculateBtpm() const -> std::vector<std::vector<std::vector<int>>>;
  void adaptBtpmToPatchCount(std::vector<std::vector<std::vector<int>>> &btpm) const;
  auto writePatchInAtlas(const MivBitstream::PatchParams &patchParams,
                         const Common::TextureDepth16Frame &view, Common::MVD16Frame &frame,
                         int frameId, size_t patchIdx) -> std::array<std::array<int64_t, 4>, 3>;
  void adaptAtlas(const MivBitstream::PatchParams &patchParams,
                  Common::TextureDepthFrame<Common::YUV400P16> &atlas, int yOcc, int xOcc,
                  const Common::Vec2i &pView, const Common::Vec2i &pAtlas) const;

  // Encoder_popFrame.cpp
  void incrementFoc();

  // TODO(#358): This is a temporary solution, to be refactored in #301 or #302
  Common::Json m_rootNode;

  // Encoder sub-components
  std::unique_ptr<DepthQualityAssessor::IDepthQualityAssessor> m_depthQualityAssessor;
  std::unique_ptr<ViewOptimizer::IViewOptimizer> m_viewOptimizer;
  std::unique_ptr<Pruner::IPruner> m_pruner;
  std::unique_ptr<Aggregator::IAggregator> m_aggregator;
  std::unique_ptr<Packer::IPacker> m_packer;
  GeometryQuantizer m_geometryQuantizer;
  GeometryDownscaler m_geometryDownscaler;
  FramePacker m_framePacker;

  Configuration m_config;

  // View-optimized encoder input
  ViewOptimizer::ViewOptimizerParams m_transportParams;
  std::vector<Common::MVD16Frame> m_transportViews;

  // Encoder output (ready for HM)
  EncoderParams m_params;
  std::deque<Common::MVD16Frame> m_videoFrameBuffer;

  // Mask aggregation state
  using NonAggregatedMask = Common::Mat<std::bitset<maxIntraPeriod>>;
  std::vector<NonAggregatedMask> m_nonAggregatedMask;
  std::vector<Common::MaskList> m_aggregatedEntityMask;
  size_t m_maxLumaSamplesPerFrame{};

  std::vector<std::vector<Common::Mat<Common::Vec3i>>> m_colorCorrectionMaps;
  std::vector<Common::Vec3i> m_patchColorCorrectionOffset;
};
} // namespace TMIV::Encoder

#endif
