/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ISO/IEC
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

#ifndef TMIV_DECODER_PRERENDERER_H
#define TMIV_DECODER_PRERENDERER_H

#include "GeometryScaler.h"

#include <TMIV/Common/Json.h>
#include <TMIV/MivBitstream/AccessUnit.h>

namespace TMIV::Decoder {
// The pre-renderer implements part of:
//
//   * ISO/IEC 23090-12 Annex A Profiles, tiers and levels
//   * ISO/IEC 23090-12 Annex B Post decoding
//   * ISO/IEC 23090-12 Annex H Rendering processes
//
// This implementation includes an alternative for Annex B.3.6 chroma up-sampling based on
// nearest-neighbour interpolation.
class PreRenderer {
public:
  explicit PreRenderer(const Common::Json &componentNode);

  void preRenderFrame(MivBitstream::AccessUnit &frame) const;

private:
  static void checkRestrictions(const MivBitstream::AccessUnit &frame);

  // ISO/IEC 23090-12 Annex B.2.1
  static void convertNominalFormat(const MivBitstream::V3cParameterSet &vps,
                                   MivBitstream::AtlasId atlasId,
                                   MivBitstream::AtlasAccessUnit &atlas);

  // ISO/IEC 23090-12 Annex B.2.2
  static void convertOccupancyNominalFormat(const MivBitstream::V3cParameterSet &vps,
                                            MivBitstream::AtlasId atlasId,
                                            MivBitstream::AtlasAccessUnit &atlas,
                                            const Common::Frame<> &inFrame);

  // ISO/IEC 23090-12 Annex B.2.3
  static void convertGeometryNominalFormat(const MivBitstream::V3cParameterSet &vps,
                                           MivBitstream::AtlasId atlasId,
                                           MivBitstream::AtlasAccessUnit &atlas,
                                           const Common::Frame<> &inFrame);

  // ISO/IEC 23090-12 Annex B.2.5
  static void convertAttributeNominalFormat(const MivBitstream::V3cParameterSet &vps,
                                            MivBitstream::AtlasId atlasId,
                                            MivBitstream::AtlasAccessUnit &atlas,
                                            const Common::FrameList<> &inFrame);

  // ISO/IEC 23090-12 Annex B.3.2
  [[nodiscard]] static auto convertBitDepth(Common::Frame<> frame, uint32_t nominalBitDepth,
                                            bool alignmentFlag, uint8_t dimensions)
      -> Common::Frame<>;

  // ISO/IEC 23090-12 Annex B.3.3
  [[nodiscard]] static auto convertResolution(Common::Frame<> inFrame, int32_t videoWidthNF,
                                              int32_t videoHeightNF) -> Common::Frame<>;

  // Alternative for ISO/IEC 23090-12 Annex B.3.7:
  //
  // * nearest neighbour interpolation
  // * bring all channels to the same resolution as the first one
  [[nodiscard]] static auto upsampleChroma(Common::Frame<> inFrame) -> Common::Frame<>;

  // ISO/IEC 23090-12 Annex B.3.9
  [[nodiscard]] static auto thresholdOccupancy(const Common::Frame<> &inFrame, uint8_t threshold)
      -> Common::Frame<bool>;

  // ISO/IEC 23090-12 Annex B.4.1
  static void unpackDecodedPackedVideo(const MivBitstream::V3cParameterSet &vps,
                                       MivBitstream::AtlasId atlasId,
                                       MivBitstream::AtlasAccessUnit &atlas);

  // ISO/IEC 23090-12 Annex B.4.2 output
  struct UnpackedVideoComponentResolutions {
    int32_t unpckOccWidth{};
    int32_t unpckOccHeight{};
    int32_t unpckGeoWidth{};
    int32_t unpckGeoHeight{};
    std::vector<int32_t> unpckAttrWidth;
    std::vector<int32_t> unpckAttrHeight;
  };

  // ISO/IEC 23090-12 Annex B.4.2
  [[nodiscard]] static auto
  calculateUnpackedVideoComponentResolution(const MivBitstream::V3cParameterSet &vps,
                                            MivBitstream::AtlasId atlasId)
      -> UnpackedVideoComponentResolutions;

  // ISO/IEC 23090-12 Annex B.4.3
  static void
  initializeUnpackedVideoComponentFrame(const MivBitstream::PackingInformation &pin,
                                        const UnpackedVideoComponentResolutions &resolutions,
                                        uint32_t bitDepth, Common::ColorFormat colorFormat,
                                        MivBitstream::AtlasAccessUnit &atlas);

  // ISO/IEC 23090-12 Annex B.4.4
  static void copyDataFromPackedRegionsToUnpackedVideoComponentFrames(
      const MivBitstream::V3cParameterSet &vps, MivBitstream::AtlasId atlasId,
      const Common::Frame<> &decPckFrameNCF, MivBitstream::AtlasAccessUnit &atlas);

  static void rescaleTexture(const MivBitstream::ViewParamsList &vpl,
                             MivBitstream::AtlasAccessUnit &atlas);

  // ISO/IEC 23090-12 Annex H.2.2
  static void reconstructOccupancy(const MivBitstream::ViewParamsList &vpl,
                                   MivBitstream::AtlasAccessUnit &atlas);
  // ISO/IEC 23090-12 Annex H.3
  void filterEntities(MivBitstream::AtlasAccessUnit &atlas) const;

  // ISO/IEC 23090-12 Annex H.4
  static void offsetTexture(const MivBitstream::V3cParameterSet &vps, MivBitstream::AtlasId atlasId,
                            MivBitstream::AtlasAccessUnit &atlas);

  // ISO/IEC 23090-12 Annex H.5
  void scaleGeometryVideo(const std::optional<MivBitstream::GeometryUpscalingParameters> &gup,
                          MivBitstream::AtlasAccessUnit &atlas) const;

  // Not specified
  void constructPixelToPatchMap(MivBitstream::AtlasAccessUnit &atlas) const;

  GeometryScaler m_geometryScaler;
  std::optional<Common::Vec2u> m_entityDecodeRange;
  int32_t m_patchMargin{};
};
} // namespace TMIV::Decoder

#endif
