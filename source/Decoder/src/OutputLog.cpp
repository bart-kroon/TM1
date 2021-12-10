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

#include <TMIV/Decoder/OutputLog.h>

#include <TMIV/Common/verify.h>

#include <fmt/format.h>
#include <fmt/ostream.h>

namespace TMIV::Decoder {
namespace {
// https://en.wikipedia.org/wiki/Cyclic_redundancy_check
namespace crc32 {
static constexpr auto table = []() {
  auto result = std::array<uint32_t, 0x100>();

  for (uint32_t i = 0; i < 0x100; ++i) {
    auto entry = i;

    for (int32_t j = 0; j < 8; ++j) {
      entry = (entry & 1) == 0 ? entry >> 1 : (entry >> 1) ^ 0xEDB88320;
    }
    Common::at(result, i) = entry;
  }
  return result;
}();

// Source of truth: https://wiki.osdev.org/CRC32
static_assert(table[0x00] == 0x00000000);
static_assert(table[0x03] == 0x990951BA);
static_assert(table[0x20] == 0x3B6E20C8);
static_assert(table[0xFF] == 0x2D02EF8D);
} // namespace crc32
} // namespace

constexpr auto HashFunction::consume(uint32_t value) noexcept -> HashFunction & {
  // Hash bytes in network order (big endian)
  for (int32_t i = 24; 0 <= i; i -= 8) {
    m_hash = (m_hash >> 8) ^ Common::at(crc32::table, (m_hash ^ (value >> i)) & 0xFF);
  }
  return *this;
}

constexpr auto HashFunction::consume(MivBitstream::ViewId value) noexcept -> HashFunction & {
  return consume(value.m_value);
}

auto HashFunction::consumeF(float value) noexcept -> HashFunction & {
  uint32_t intValue{};
  memcpy(&intValue, &value, 4);
  return consume(intValue);
}

constexpr auto HashFunction::result() const noexcept { return ~m_hash; }

auto HashFunction::toString(uint32_t value) -> std::string {
  return fmt::format(FMT_STRING("{:08x}"), value);
}

auto videoDataHash(const MivBitstream::AtlasAccessUnit &frame) noexcept -> HashFunction::Result {
  auto hash = HashFunction{};

  if (!frame.decGeoFrame.empty()) {
    for (auto x : frame.decGeoFrame.getPlane(0)) {
      hash.consume(x);
    }
  }

  if (!frame.decOccFrame.empty()) {
    for (auto x : frame.decOccFrame.getPlane(0)) {
      hash.consume(x);
    }
  }

  for (const auto &x : frame.decAttrFrame) {
    for (const auto &y : x.getPlanes()) {
      for (auto z : y) {
        hash.consume(z);
      }
    }
  }

  return hash.result();
}

auto blockToPatchMapHash(const MivBitstream::AtlasAccessUnit &frame) noexcept
    -> HashFunction::Result {
  auto hash = HashFunction{};

  for (auto x : frame.blockToPatchMap.getPlane(0)) {
    hash.consume(x == Common::unusedPatchId ? 0xFFFF'FFFF : x);
  }
  return hash.result();
}

auto patchParamsListHash(const MivBitstream::PatchParamsList &ppl) noexcept
    -> HashFunction::Result {
  auto hash = HashFunction{};

  for (const auto &pp : ppl) {
    hash.consume(pp.atlasPatch3dOffsetU());
    hash.consume(pp.atlasPatch3dOffsetV());
    hash.consume(pp.atlasPatch3dOffsetD());
    hash.consume(pp.atlasPatch3dRangeD());
    hash.consume(pp.atlasPatchProjectionId());
    hash.consume(static_cast<uint32_t>(pp.atlasPatchOrientationIndex()));
    hash.consume(pp.atlasPatchLoDScaleX());
    hash.consume(pp.atlasPatchLoDScaleY());
    hash.consume(pp.atlasPatchEntityId());
    hash.consume(pp.atlasPatchDepthOccThreshold());
    hash.consume(pp.atlasPatchAttributeOffset()[0]);
    hash.consume(pp.atlasPatchAttributeOffset()[1]);
    hash.consume(pp.atlasPatchAttributeOffset()[2]);
    hash.consume(static_cast<uint32_t>(pp.atlasPatchInpaintFlag()));
  }
  return hash.result();
}

auto viewParamsListHash(const MivBitstream::ViewParamsList &vpl) noexcept -> HashFunction::Result {
  auto hash = HashFunction{};

  for (const auto &vp : vpl) {
    hash.consume(static_cast<uint32_t>(vp.ci.ci_cam_type()));
    hash.consume(vp.ci.ci_projection_plane_width_minus1());
    hash.consume(vp.ci.ci_projection_plane_height_minus1());

    switch (vp.ci.ci_cam_type()) {
    case MivBitstream::CiCamType::equirectangular:
      hash.consumeF(vp.ci.ci_erp_phi_min());
      hash.consumeF(vp.ci.ci_erp_phi_max());
      hash.consumeF(vp.ci.ci_erp_theta_min());
      hash.consumeF(vp.ci.ci_erp_theta_max());
      break;
    case MivBitstream::CiCamType::perspective:
      hash.consumeF(vp.ci.ci_perspective_focal_hor());
      hash.consumeF(vp.ci.ci_perspective_focal_ver());
      hash.consumeF(vp.ci.ci_perspective_center_hor());
      hash.consumeF(vp.ci.ci_perspective_center_ver());
      break;
    case MivBitstream::CiCamType::orthographic:
      hash.consumeF(vp.ci.ci_ortho_width());
      hash.consumeF(vp.ci.ci_ortho_height());
      break;
    }

    hash.consumeF(vp.pose.position[0]);
    hash.consumeF(vp.pose.position[1]);
    hash.consumeF(vp.pose.position[2]);

    // orientation = w + i x + j y + k z
    hash.consumeF(static_cast<float>(vp.pose.orientation[3])); // w
    hash.consumeF(static_cast<float>(vp.pose.orientation[0])); // x
    hash.consumeF(static_cast<float>(vp.pose.orientation[1])); // y
    hash.consumeF(static_cast<float>(vp.pose.orientation[2])); // z

    hash.consumeF(vp.dq.dq_norm_disp_low());
    hash.consumeF(vp.dq.dq_norm_disp_high());
    hash.consume(vp.dq.dq_depth_occ_threshold_default());
  }

  return hash.result();
}

void writeFrameToOutputLog(const MivBitstream::AccessUnit &frame, std::ostream &stream) {
  const auto vplHashString = HashFunction::toString(viewParamsListHash(frame.viewParamsList));

  for (uint8_t k = 0; k <= frame.vps.vps_atlas_count_minus1(); ++k) {
    fmt::print(
        stream, FMT_STRING("{} {} {} {} {} {} {} {}\n"), frame.foc, frame.vps.vps_atlas_id(k),
        frame.atlas[k].asps.asps_frame_width(), frame.atlas[k].asps.asps_frame_height(),
        HashFunction::toString(videoDataHash(frame.atlas[k])),
        HashFunction::toString(blockToPatchMapHash(frame.atlas[k])),
        HashFunction::toString(patchParamsListHash(frame.atlas[k].patchParamsList)), vplHashString);
  }
}
} // namespace TMIV::Decoder
