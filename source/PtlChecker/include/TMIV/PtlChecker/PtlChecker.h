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

#ifndef TMIV_PTLCHECKER_PTLCHECKER_H
#define TMIV_PTLCHECKER_PTLCHECKER_H

#include <TMIV/PtlChecker/AbstractChecker.h>

namespace TMIV::PtlChecker {
class PtlChecker : public AbstractChecker {
public:
  using Logger = std::function<void(const std::string &failure)>;

  // To support experimentation, the PTL checker only logs a warning. This behaviour can be changed
  // by replacing the log function.
  void replaceLogger(Logger value) override;

  void checkVuh(const MivBitstream::V3cUnitHeader &vuh) override;
  void checkAndActivateNuh(const MivBitstream::NalUnitHeader &nuh) override;
  void checkAndActivateVps(const MivBitstream::V3cParameterSet &vps) override;
  void checkAndActivateAsps(MivBitstream::AtlasId atlasId,
                            const MivBitstream::AtlasSequenceParameterSetRBSP &asps) override;
  void checkAfps(const MivBitstream::AtlasFrameParameterSetRBSP &afps) override;
  void checkAtl(const MivBitstream::AtlasTileLayerRBSP &atl) override;
  void checkCaf(const MivBitstream::CommonAtlasFrameRBSP &caf) override;
  void checkVideoFrame(MivBitstream::VuhUnitType vut, const Common::Frame<> &frame) override;

private:
  static void defaultLogger(const std::string &failure);

  [[nodiscard]] auto ptl_profile_codec_group_idc() const noexcept;
  [[nodiscard]] auto ptl_profile_toolset_idc() const noexcept;
  [[nodiscard]] auto ptl_profile_reconstruction_idc() const noexcept;
  [[nodiscard]] auto ptl_tier_flag() const noexcept;
  [[nodiscard]] auto ptl_level_idc() const noexcept;
  [[nodiscard]] auto ptc_restricted_geometry_flag() const noexcept;

  [[nodiscard]] auto maxAtlasSize() const noexcept;
  [[nodiscard]] auto levelMapCount() const noexcept;
  [[nodiscard]] auto maxNumAttributeCount() const noexcept;

  void checkVpsCommon(const MivBitstream::V3cParameterSet &vps) const;
  void checkVpsAtlas(const MivBitstream::V3cParameterSet &vps, MivBitstream::AtlasId atlasId) const;
  void checkGeometryInformation(const MivBitstream::GeometryInformation &gi) const;
  void checkAttributesInformation(const MivBitstream::AttributeInformation &ai) const;
  void checkAttributeInformation(const MivBitstream::AttributeInformation &ai,
                                 uint8_t attrIdx) const;
  void checkVpsMivExtension(const MivBitstream::VpsMivExtension &vme) const;

  void checkAsme(MivBitstream::AtlasId atlasId, const MivBitstream::AspsMivExtension &asme) const;

  void checkOccupancyVideoFrame(const Common::Frame<> &frame) const;
  void checkGeometryVideoFrame(const Common::Frame<> &frame) const;
  void checkAttributeVideoFrame(const Common::Frame<> &frame) const;

  Logger m_logger{&defaultLogger};
  std::optional<MivBitstream::V3cParameterSet> m_vps;
  std::optional<MivBitstream::AtlasSequenceParameterSetRBSP> m_asps;
  std::optional<MivBitstream::NalUnitHeader> m_nuh;
};
} // namespace TMIV::PtlChecker

#endif
