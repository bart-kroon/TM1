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

#ifndef TMIV_ENCODER_CONFIGURATION_H
#define TMIV_ENCODER_CONFIGURATION_H

#include <TMIV/Common/Json.h>
#include <TMIV/MivBitstream/V3cParameterSet.h>
#include <TMIV/MivBitstream/ViewingSpace.h>
#include <TMIV/MivBitstream/ViewportCameraParameters.h>
#include <TMIV/MivBitstream/ViewportPosition.h>

namespace TMIV::Encoder {
static constexpr auto maxIntraPeriod = 32;

struct Configuration {
  Configuration(const Common::Json & /*rootNode*/, const Common::Json & /*componentNode*/);

  // Main parameters
  int32_t intraPeriod;
  Common::Vec2i blockSizeDepthQualityDependent;
  std::optional<bool> depthLowQualityFlag;
  double maxLumaSampleRate{};
  int32_t maxLumaPictureSize{};
  int32_t maxAtlases{};
  bool haveTexture;
  bool haveGeometry;
  bool haveOccupancy;
  bool framePacking;
  bool oneViewPerAtlasFlag;
  std::vector<Common::Vec2i> overrideAtlasFrameSizes{};
  double depthOccThresholdIfSet{};
  bool geometryScaleEnabledFlag;
  int32_t dilationIter;
  Common::stack::Vec2<Common::SampleValue> entityEncRange;
  bool dynamicDepthRange;
  bool attributeOffsetFlag;
  uint32_t attributeOffsetBitCount{};
  bool dqParamsPresentFlag{true};
  bool colorCorrectionEnabledFlag;
  bool randomAccess;
  bool patchRedundancyRemoval;
  uint8_t numGroups;
  uint16_t maxEntityId{};

  // SEI-related parameters
  bool viewportCameraParametersSei;
  bool viewportPositionSei;
  std::optional<MivBitstream::ViewingSpace> viewingSpace;

  // Profile-tier-level parameters
  MivBitstream::PtlProfileCodecGroupIdc codecGroupIdc{};
  MivBitstream::PtlProfileToolsetIdc toolsetIdc{};
  MivBitstream::PtlProfileReconstructionIdc reconstructionIdc{};
  MivBitstream::PtlLevelIdc levelIdc{};

  // Bit-depth parameters
  uint32_t occBitDepth{};
  uint32_t geoBitDepth{};
  uint32_t texBitDepth{};
  uint32_t pacBitDepth{};

  [[nodiscard]] auto blockSize(bool depthLowQualityFlag_) const noexcept {
    return blockSizeDepthQualityDependent[static_cast<int32_t>(depthLowQualityFlag_)];
  }

private:
  void queryMainParameters(const Common::Json &rootNode, const Common::Json &componentNode);
  void queryProfileTierLevelParameters(const Common::Json &rootNode);
  void queryBitDepthParameters(const Common::Json &rootNode);
  void querySeiParameters(const Common::Json &rootNode);
  void verifyValid() const;
};
} // namespace TMIV::Encoder

#endif
