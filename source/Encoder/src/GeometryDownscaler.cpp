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

#include <TMIV/Encoder/GeometryDownscaler.h>

namespace TMIV::Encoder {
namespace {
auto maxPool(const Common::Frame<> &frame, Common::Vec2i frameSize) -> Common::Frame<> {
  auto result = Common::Frame<>::lumaOnly(frameSize, frame.getBitDepth());

  for (int32_t y = 0; y < frameSize.y(); ++y) {
    const int32_t i1 = y * frame.getHeight() / frameSize.y();
    const int32_t i2 = (y + 1) * frame.getHeight() / frameSize.y();

    for (int32_t x = 0; x < frameSize.x(); ++x) {
      const int32_t j1 = x * frame.getWidth() / frameSize.x();
      const int32_t j2 = (x + 1) * frame.getWidth() / frameSize.x();

      auto maximum = uint16_t{};

      for (int32_t i = i1; i < i2; ++i) {
        for (int32_t j = j1; j < j2; ++j) {
          maximum = std::max(maximum, frame.getPlane(0)(i, j));
        }
      }

      result.getPlane(0)(y, x) = maximum;
    }
  }

  return result;
}
} // namespace

auto GeometryDownscaler::transformFrame(const std::vector<EncoderAtlasParams> &atlas,
                                        Common::V3cFrameList frame) -> Common::V3cFrameList {
  for (size_t k = 0; k < atlas.size(); ++k) {
    const auto &asps = atlas[k].asps;

    auto frameSize = Common::Vec2i{asps.asps_frame_width(), asps.asps_frame_height()};
    const auto &asme = asps.asps_miv_extension();

    if (asme.asme_geometry_scale_enabled_flag()) {
      frameSize.x() /= asme.asme_geometry_scale_factor_x_minus1() + 1;
      frameSize.y() /= asme.asme_geometry_scale_factor_y_minus1() + 1;
    }

    frame[k].geometry = maxPool(frame[k].geometry, frameSize);
  }
  return frame;
}
} // namespace TMIV::Encoder
