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

#include <TMIV/Encoder/FilterPatchMarginStage.h>

namespace TMIV::Encoder {
namespace {
void clearPatchMargins(CodableUnit &unit, size_t k, Common::Frame<> &tmpTex,
                       Common::Frame<> &tmpGeo, Common::Frame<> &tmpTmp) {
  auto &atlas = unit.deepFrameList[k];

  tmpTex.createYuv420(atlas.texture.getSize(), atlas.texture.getBitDepth());
  tmpGeo.createY(atlas.geometry.getSize(), atlas.geometry.getBitDepth());
  tmpTmp.createY(atlas.geometry.getSize(), 8);

  tmpTex.fillNeutral();
  tmpGeo.fillZero();
  tmpTmp.fillOne();

  for (const auto &patch : unit.encoderParams.patchParamsList) {
    if (k != unit.encoderParams.vps.indexOf(patch.atlasId())) {
      continue;
    }

    auto x1 = patch.atlasPatch2dPosX();
    auto y1 = patch.atlasPatch2dPosY();
    auto x2 = patch.atlasPatch2dPosX() + patch.atlasPatch2dSizeX();
    auto y2 = patch.atlasPatch2dPosY() + patch.atlasPatch2dSizeY();

    for (int32_t y = y1; y < y2; ++y) {
      for (int32_t x = x1; x < x2; ++x) {
        tmpTex.getPlane(0)(y, x) = 0;
        tmpTex.getPlane(1)(y / 2, x / 2) = 0;
        tmpTex.getPlane(2)(y / 2, x / 2) = 0;
        tmpGeo.getPlane(0)(y, x) = 0;
        tmpTmp.getPlane(0)(y, x) = 0;
      }
    }

    x1 += patch.isRotated() ? patch.atlasPatch2dMarginV() : patch.atlasPatch2dMarginU();
    y1 += patch.isRotated() ? patch.atlasPatch2dMarginU() : patch.atlasPatch2dMarginV();
    x2 -= patch.isRotated() ? patch.atlasPatch2dMarginV() : patch.atlasPatch2dMarginU();
    y2 -= patch.isRotated() ? patch.atlasPatch2dMarginU() : patch.atlasPatch2dMarginV();

    for (int32_t y = y1; y < y2; ++y) {
      for (int32_t x = x1; x < x2; ++x) {
        tmpTex.getPlane(0)(y, x) = atlas.texture.getPlane(0)(y, x);
        tmpTex.getPlane(1)(y / 2, x / 2) = atlas.texture.getPlane(1)(y / 2, x / 2);
        tmpTex.getPlane(2)(y / 2, x / 2) = atlas.texture.getPlane(2)(y / 2, x / 2);
        tmpGeo.getPlane(0)(y, x) = atlas.geometry.getPlane(0)(y, x);
        tmpTmp.getPlane(0)(y, x) = 1;
      }
    }
  }

  atlas.texture.getPlane(0) = tmpTex.getPlane(0);
}

void inpaintPatchMargins(CodableUnit &unit, size_t k, Common::Frame<> &tmpTex,
                         Common::Frame<> &tmpGeo, Common::Frame<> &tmpTmp) {
  auto &atlas = unit.deepFrameList[k];

  const auto W = tmpTex.getWidth();
  const auto H = tmpTex.getHeight();
  const auto depthLowQualityFlag =
      unit.encoderParams.casps.casps_miv_extension().casme_depth_low_quality_flag();

  for (int32_t i = 0; i < 32; i++) {
    for (auto h = 0; h < H; h++) {
      for (auto w = 0; w < W; w++) {
        tmpTex.getPlane(0)(h, w) = 0;
        int32_t valY = 0;
        int32_t valU = 0;
        int32_t valV = 0;
        int32_t valD = 0;
        int32_t cnt = 0;
        auto m = i;
        if (atlas.texture.getPlane(0)(h, w) != 0U) {
          continue;
        }
        for (auto hh = std::max(0, h - m); hh <= std::min(h + m, H - 1); hh++) {
          for (auto ww = std::max(0, w - m); ww <= std::min(w + m, W - 1); ww++) {
            if (atlas.texture.getPlane(0)(hh, ww) != 0U) {
              valY += static_cast<int32_t>(atlas.texture.getPlane(0)(hh, ww));
              valU += static_cast<int32_t>(atlas.texture.getPlane(1)(hh / 2, ww / 2));
              valV += static_cast<int32_t>(atlas.texture.getPlane(2)(hh / 2, ww / 2));
              valD += static_cast<int32_t>(atlas.geometry.getPlane(0)(hh, ww));
              cnt++;
            }
          }
        }
        if (cnt != 0U) {
          tmpTex.getPlane(0)(h, w) = static_cast<uint16_t>(valY / cnt);
          tmpTex.getPlane(1)(h / 2, w / 2) = static_cast<uint16_t>(valU / cnt);
          tmpTex.getPlane(2)(h / 2, w / 2) = static_cast<uint16_t>(valV / cnt);
          tmpGeo.getPlane(0)(h, w) = static_cast<uint16_t>(valD / cnt);
          tmpTmp.getPlane(0)(h, w) = static_cast<uint16_t>(m + 1);
        }
      }
    }
    for (auto h = 0; h < H; h++) {
      for (auto w = 0; w < W; w++) {
        if (tmpTex.getPlane(0)(h, w) != 0U && atlas.texture.getPlane(0)(h, w) == 0U) {
          atlas.texture.getPlane(0)(h, w) = tmpTex.getPlane(0)(h, w);
          atlas.texture.getPlane(1)(h / 2, w / 2) = tmpTex.getPlane(1)(h / 2, w / 2);
          atlas.texture.getPlane(2)(h / 2, w / 2) = tmpTex.getPlane(2)(h / 2, w / 2);
          if (depthLowQualityFlag) {
            atlas.geometry.getPlane(0)(h, w) = tmpGeo.getPlane(0)(h, w);
          }
        }
      }
    }
  }
}

void blurPatchMargins(CodableUnit &unit, size_t k, Common::Frame<> &tmpTex, Common::Frame<> &tmpGeo,
                      Common::Frame<> &tmpTmp) {
  auto &atlas = unit.deepFrameList[k];

  const auto W = tmpTex.getWidth();
  const auto H = tmpTex.getHeight();
  const auto depthLowQualityFlag =
      unit.encoderParams.casps.casps_miv_extension().casme_depth_low_quality_flag();

  for (auto h = 0; h < H; h++) {
    for (auto w = 0; w < W; w++) {
      int32_t valY = 0;
      int32_t valU = 0;
      int32_t valV = 0;
      int32_t valD = 0;
      int32_t cnt = 0;
      int32_t m = tmpTmp.getPlane(0)(h, w) - 1;
      for (auto hh = std::max(0, h - m); hh <= std::min(h + m, H - 1); hh++) {
        for (auto ww = std::max(0, w - m); ww <= std::min(w + m, W - 1); ww++) {
          valY += static_cast<int32_t>(atlas.texture.getPlane(0)(hh, ww));
          valU += static_cast<int32_t>(atlas.texture.getPlane(1)(hh / 2, ww / 2));
          valV += static_cast<int32_t>(atlas.texture.getPlane(2)(hh / 2, ww / 2));
          valD += static_cast<int32_t>(atlas.geometry.getPlane(0)(hh, ww));
          cnt++;
        }
      }
      tmpTex.getPlane(0)(h, w) = static_cast<uint16_t>(valY / cnt);
      tmpTex.getPlane(1)(h / 2, w / 2) = static_cast<uint16_t>(valU / cnt);
      tmpTex.getPlane(2)(h / 2, w / 2) = static_cast<uint16_t>(valV / cnt);
      tmpGeo.getPlane(0)(h, w) = static_cast<uint16_t>(valD / cnt);
    }
  }
  for (auto h = 0; h < H; h++) {
    for (auto w = 0; w < W; w++) {
      atlas.texture.getPlane(0)(h, w) = tmpTex.getPlane(0)(h, w);
      atlas.texture.getPlane(1)(h / 2, w / 2) = tmpTex.getPlane(1)(h / 2, w / 2);
      atlas.texture.getPlane(2)(h / 2, w / 2) = tmpTex.getPlane(2)(h / 2, w / 2);
      if (depthLowQualityFlag) {
        atlas.geometry.getPlane(0)(h, w) = tmpGeo.getPlane(0)(h, w);
      }
    }
  }
}
} // namespace

FilterPatchMarginStage::FilterPatchMarginStage(const Common::Json &componentNode)
    : m_patchMarginEnabledFlag{componentNode.require("haveTextureVideo").as<bool>() &&
                               componentNode.require("patchMarginEnabledFlag").as<bool>()} {}

void FilterPatchMarginStage::encode(CodableUnit unit) {
  Common::logVerbose("Filter patch margin stage");

  auto tmpTex = Common::Frame{};
  auto tmpGeo = Common::Frame{};
  auto tmpTmp = Common::Frame{};

  if (m_patchMarginEnabledFlag) {
    unit.encoderParams.vps.vps_miv_2_extension().vme_patch_margin_enabled_flag(true);

    for (size_t k = 0; k < unit.deepFrameList.size(); ++k) {
      unit.encoderParams.atlas[k].asps.asps_miv_2_extension().asme_patch_margin_enabled_flag(true);

      clearPatchMargins(unit, k, tmpTex, tmpGeo, tmpTmp);
      inpaintPatchMargins(unit, k, tmpTex, tmpGeo, tmpTmp);
      blurPatchMargins(unit, k, tmpTex, tmpGeo, tmpTmp);
    }
  }

  source.encode(unit);
}

} // namespace TMIV::Encoder
