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

#include <TMIV/Decoder/GeometryScaler.h>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <utility>

namespace TMIV::Decoder {
namespace {
auto getNeighborhood5() -> std::vector<Common::Vec2i> {
  return {Common::Vec2i{0, 0}, Common::Vec2i{0, -1}, Common::Vec2i{1, 0}, Common::Vec2i{0, 1},
          Common::Vec2i{-1, 0}};
}

auto getNeighborhood3x3() -> std::vector<Common::Vec2i> {
  return {Common::Vec2i{0, 0},  Common::Vec2i{0, -1}, Common::Vec2i{1, -1},
          Common::Vec2i{1, 0},  Common::Vec2i{1, 1},  Common::Vec2i{0, 1},
          Common::Vec2i{-1, 1}, Common::Vec2i{-1, 0}, Common::Vec2i{-1, -1}};
}

auto getNeighborhood5x5() -> std::vector<Common::Vec2i> {
  return {Common::Vec2i{0, 0},   Common::Vec2i{0, -1}, Common::Vec2i{1, -1},  Common::Vec2i{1, 0},
          Common::Vec2i{1, 1},   Common::Vec2i{0, 1},  Common::Vec2i{-1, 1},  Common::Vec2i{-1, 0},
          Common::Vec2i{-1, -1}, Common::Vec2i{0, -2}, Common::Vec2i{1, -2},  Common::Vec2i{2, -2},
          Common::Vec2i{2, -1},  Common::Vec2i{2, 0},  Common::Vec2i{2, 1},   Common::Vec2i{2, 2},
          Common::Vec2i{1, 2},   Common::Vec2i{0, 2},  Common::Vec2i{-1, 2},  Common::Vec2i{-2, 2},
          Common::Vec2i{-2, 1},  Common::Vec2i{-2, 0}, Common::Vec2i{-2, -1}, Common::Vec2i{-2, -2},
          Common::Vec2i{-1, -2}};
}

template <typename T>
auto sampleKernel(const Common::Mat<T> &mat, const Common::Vec2i &loc,
                  const std::vector<Common::Vec2i> &kernelPoints) {
  std::vector<T> samples(kernelPoints.size());

  auto s = samples.begin();
  for (const auto &pnt : kernelPoints) {
    auto loc_k = loc + pnt;
    *s++ = mat(loc_k[1], loc_k[0]);
  }

  return samples;
}

auto sampleKernel(const Common::Texture444Frame &attrFrame, const Common::Vec2i &loc,
                  const std::vector<Common::Vec2i> &kernelPoints) {
  auto channels = std::array<std::vector<uint16_t>, 3>{};

  for (int32_t d = 0; d < 3; ++d) {
    Common::at(channels, d) = sampleKernel(attrFrame.getPlane(d), loc, kernelPoints);
  }

  auto samples = std::vector<Common::Vec3w>(channels.front().size());

  for (size_t i = 0; i < channels.front().size(); ++i) {
    for (int32_t d = 0; d < 3; ++d) {
      samples[i][d] = Common::at(channels, d)[i];
    }
  }

  return samples;
}

auto minMasked(const std::vector<uint16_t> &values, const std::vector<uint8_t> &mask) {
  auto minValue = values[0]; // original foreground value
  PRECONDITION(mask[0] != 0);

  auto m = mask.begin();
  for (const auto &v : values) {
    if (*m++ == 0) {
      minValue = std::min(v, minValue);
    }
  }

  return minValue;
}

inline auto colorDistance(const Common::Vec3w &a, const Common::Vec3w &b) {
  return static_cast<float>(std::abs(a[0] - b[0]) + std::abs(a[1] - b[1]) + std::abs(a[2] - b[2]));
}

template <typename Range>
auto meanColorDistance(const Common::Vec3w &color, const Range &rangeOfColors) {
  const auto N = static_cast<uint32_t>(rangeOfColors.size());
  PRECONDITION(N > 0U);

  float meanDistance = 0.0F;
  for (auto &colorInRange : rangeOfColors) {
    meanDistance += colorDistance(color, colorInRange);
  }

  return meanDistance / static_cast<float>(N);
}

auto findForegroundEdges(const Common::Mat<uint16_t> &depth) -> Common::Mat<uint8_t> {
  auto edgeMask = Common::Mat<uint8_t>{depth.sizes()};
  auto m_kernelPoints = getNeighborhood3x3();
  for (int32_t i = 1; i < static_cast<int32_t>(depth.height()) - 1; ++i) {
    for (int32_t j = 1; j < static_cast<int32_t>(depth.width()) - 1; ++j) {
      const auto s = sampleKernel(depth, Common::Vec2i{j, i}, m_kernelPoints);

      auto e4 = Common::Vec4i{s[0] - s[1], s[0] - s[3], s[0] - s[5], s[0] - s[7]};
      auto m = std::max(e4[0], std::max(e4[1], std::max(e4[2], e4[3])));

      edgeMask(i, j) = static_cast<uint8_t>(std::min(255, m));
    }
  }
  return edgeMask;
}

auto findRegionBoundaries(const Common::Mat<uint16_t> &regionLabels) -> Common::Mat<uint8_t> {
  auto boundaryMask = Common::Mat<uint8_t>{regionLabels.sizes()};
  auto m_kernelPoints = getNeighborhood5();
  for (int32_t i = 1; i < static_cast<int32_t>(regionLabels.height()) - 1; ++i) {
    for (int32_t j = 1; j < static_cast<int32_t>(regionLabels.width()) - 1; ++j) {
      const auto s = sampleKernel(regionLabels, Common::Vec2i{j, i}, m_kernelPoints);
      bool sameRegion = s[0] == s[1] && s[0] == s[2] && s[0] == s[3] && s[0] == s[4];
      boundaryMask(i, j) = sameRegion ? 0 : 255;
    }
  }
  return boundaryMask;
}

auto findForegroundEdges(const Common::Mat<uint16_t> &depth,
                         const Common::Mat<uint16_t> &regionLabels) -> Common::Mat<uint8_t> {
  auto edges = findForegroundEdges(depth);
  const auto bounds = findRegionBoundaries(regionLabels);
  std::transform(std::begin(edges), std::end(edges), std::begin(bounds), std::begin(edges),
                 [](uint8_t e, uint8_t b) { return b != 0 ? uint8_t{} : e; });
  return edges;
}

auto erodeMasked(const Common::Mat<uint16_t> &depth, const Common::Mat<uint8_t> &mask)
    -> Common::Mat<uint16_t> {
  auto depthOut = depth;
  auto kernelPoints = getNeighborhood3x3();
  for (int32_t i = 1; i < static_cast<int32_t>(depth.height()) - 1; ++i) {
    for (int32_t j = 1; j < static_cast<int32_t>(depth.width()) - 1; ++j) {
      if (mask(i, j) != 0) {
        const auto depthSamples = sampleKernel(depth, Common::Vec2i{j, i}, kernelPoints);
        const auto maskSamples = sampleKernel(mask, Common::Vec2i{j, i}, kernelPoints);
        auto depthEroded = minMasked(depthSamples, maskSamples);
        depthOut(i, j) = depthEroded;
      }
    }
  }
  return depthOut;
}

class DepthMapAlignerColorBased {
public:
  DepthMapAlignerColorBased(int32_t geometryEdgeMagnitudeTh, float minForegroundConfidence)
      : m_geometryEdgeMagnitudeTh{geometryEdgeMagnitudeTh}
      , m_minForegroundConfidence{minForegroundConfidence}
      , m_kernelPoints{getNeighborhood5x5()} {}

  [[nodiscard]] auto colorConfidence(const std::vector<uint16_t> &depthValues,
                                     const std::vector<Common::Vec3w> &colorValues,
                                     const std::vector<uint8_t> &edgeMagnitudes) const -> float {
    const auto N = depthValues.size();
    PRECONDITION(N == colorValues.size());
    PRECONDITION(N == edgeMagnitudes.size());

    const int32_t depthCentral = depthValues[0];
    const int32_t depthLow = depthCentral - m_geometryEdgeMagnitudeTh;
    const int32_t depthHigh = depthCentral + m_geometryEdgeMagnitudeTh;
    const auto colorCentral = colorValues[0];

    // split colors samples in kernel in foreground and background
    // exclude the (uncertain) depth edges and pixels beyond foreground
    std::vector<Common::Vec3w> colorsFG;
    std::vector<Common::Vec3w> colorsBG;
    for (auto i = 1U; i < N; ++i) {
      if (edgeMagnitudes[i] < m_geometryEdgeMagnitudeTh && depthValues[i] < depthHigh) {
        if (depthValues[i] > depthLow) {
          colorsFG.push_back(colorValues[i]);
        } else {
          colorsBG.push_back(colorValues[i]);
        }
      }
    }

    // make the groups of equal size
    int32_t groupSize = static_cast<int32_t>(std::min(colorsFG.size(), colorsBG.size()));

    float foregroundColorConfidence = 1.F;
    if (groupSize > 0) {
      colorsFG.resize(groupSize);
      colorsBG.resize(groupSize);
      auto meanDistanceBG = meanColorDistance(colorCentral, colorsBG);
      auto meanDistanceFG = meanColorDistance(colorCentral, colorsFG);

      // the confidence the edge belongs to foreground
      if (meanDistanceFG + meanDistanceBG > 0.F) {
        foregroundColorConfidence = meanDistanceBG / (meanDistanceFG + meanDistanceBG);
      }
    }

    return foregroundColorConfidence;
  }

  auto colorConfidenceAt(const Common::Texture444Frame &attrFrame,
                         const Common::Mat<uint16_t> &depth,
                         const Common::Mat<uint8_t> &edgeMagnitudes, const Common::Vec2i &loc)
      -> float {
    auto depths = sampleKernel(depth, loc, m_kernelPoints);
    auto colors = sampleKernel(attrFrame, loc, m_kernelPoints);
    auto edges = sampleKernel(edgeMagnitudes, loc, m_kernelPoints);

    return colorConfidence(depths, colors, edges);
  }

  auto operator()(const Common::Texture444Frame &attrFrame, const Common::Mat<uint16_t> &depth,
                  const Common::Mat<uint8_t> &edgeMagnitudes) -> Common::Mat<uint16_t> {
    const int32_t numIterations = 1;
    auto depthIter = depth;
    for (int32_t iter = 0; iter < numIterations; iter++) {
      auto markers = Common::Mat<uint8_t>{depth.sizes()};
      for (int32_t i = m_B; i < static_cast<int32_t>(depth.height()) - m_B; ++i) {
        for (int32_t j = m_B; j < static_cast<int32_t>(depth.width()) - m_B; ++j) {
          if (edgeMagnitudes(i, j) >= m_geometryEdgeMagnitudeTh) {
            auto foregroundConfidence =
                colorConfidenceAt(attrFrame, depthIter, edgeMagnitudes, {j, i});
            if (foregroundConfidence < m_minForegroundConfidence) {
              markers(i, j) = 255;
            }
          }
        }
      }
      depthIter = erodeMasked(depthIter, markers);
    }
    return depthIter;
  }

private:
  int32_t m_geometryEdgeMagnitudeTh;
  float m_minForegroundConfidence;
  std::vector<Common::Vec2i> m_kernelPoints;
  int32_t m_B = 2;
};

class DepthMapAlignerCurvatureBased {
public:
  DepthMapAlignerCurvatureBased(int32_t geometryEdgeMagnitudeTh, int32_t maxCurvature)
      : m_geometryEdgeMagnitudeTh(geometryEdgeMagnitudeTh)
      , m_maxCurvature(maxCurvature)
      , m_kernelPoints{getNeighborhood3x3()} {}

  [[nodiscard]] auto curvature(const std::vector<uint16_t> &depthValues) const -> int32_t {
    const int32_t depthCentral = depthValues[0];
    const int32_t depthLow = depthCentral - m_geometryEdgeMagnitudeTh;

    int32_t depthCurvature3x3 = 0;
    for (size_t i = 1; i < depthValues.size(); ++i) {
      if (int32_t{depthValues[i]} < depthLow) {
        depthCurvature3x3++;
      }
    }

    return depthCurvature3x3;
  }

  auto curvatureAt(const Common::Mat<uint16_t> &depth, const Common::Vec2i &loc) -> int32_t {
    auto depths = sampleKernel(depth, loc, m_kernelPoints);

    return curvature(depths);
  }

  auto operator()(const Common::Mat<uint16_t> &depth, const Common::Mat<uint8_t> &edgeMagnitudes)
      -> Common::Mat<uint16_t> {
    auto depthOut = depth;
    auto markers = Common::Mat<uint8_t>{depth.sizes()};

    for (int32_t i = m_B; i < static_cast<int32_t>(depth.height()) - m_B; ++i) {
      for (int32_t j = m_B; j < static_cast<int32_t>(depth.width()) - m_B; ++j) {
        if (edgeMagnitudes(i, j) >= m_geometryEdgeMagnitudeTh) {
          auto curvature = curvatureAt(depth, {j, i});
          if (curvature >= m_maxCurvature) {
            markers(i, j) = 255;
          }
        }
      }
    }

    depthOut = erodeMasked(depth, markers);

    return depthOut;
  }

private:
  int32_t m_geometryEdgeMagnitudeTh = 11;
  int32_t m_maxCurvature = 6;
  std::vector<Common::Vec2i> m_kernelPoints;
  int32_t m_B = 1;
};

auto upscaleNearest(const Common::Mat<uint16_t> &input, Common::Vec2i outputSize)
    -> Common::Mat<uint16_t> {
  const auto inputSize =
      Common::Vec2i{static_cast<int32_t>(input.width()), static_cast<int32_t>(input.height())};
  auto output = Common::Mat<uint16_t>(
      {static_cast<size_t>(outputSize.y()), static_cast<size_t>(outputSize.x())});

  for (int32_t yo = 0; yo < outputSize.y(); ++yo) {
    for (int32_t xo = 0; xo < outputSize.x(); ++xo) {
      const auto xi = xo * inputSize.x() / outputSize.x();
      const auto yi = yo * inputSize.y() / outputSize.y();
      output(yo, xo) = input(yi, xi);
    }
  }

  return output;
}

class DepthUpscaler {
public:
  DepthUpscaler(int32_t geometryEdgeMagnitudeTh, float minForegroundConfidence,
                int32_t maxCurvature)
      : m_alignerColor(geometryEdgeMagnitudeTh, minForegroundConfidence)
      , m_alignerCurvature(geometryEdgeMagnitudeTh, maxCurvature) {}

  auto operator()(const MivBitstream::AtlasAccessUnit &atlas) -> Common::Depth10Frame {
    auto geoFrame =
        Common::Depth10Frame::lumaOnly(atlas.frameSize(), atlas.decGeoFrame.getBitDepth());

    // Upscale with nearest neighbor interpolation to nominal atlas resolution
    const auto depthUpscaled = upscaleNearest(atlas.decGeoFrame.getPlane(0), atlas.frameSize());
    const auto regionsUpscaled =
        upscaleNearest(atlas.blockToPatchMap.getPlane(0), atlas.frameSize());

    // Erode based on color alignment
    const auto edgeMagnitudes1 = findForegroundEdges(depthUpscaled, regionsUpscaled);
    const auto depthColorAligned = m_alignerColor(atlas.attrFrame, depthUpscaled, edgeMagnitudes1);

    // Erode based on (fg) curvature
    const auto edgeMagnitudes2 = findForegroundEdges(depthColorAligned, regionsUpscaled);
    geoFrame.getPlane(0) = m_alignerCurvature(depthColorAligned, edgeMagnitudes2);
    return geoFrame;
  }

private:
  DepthMapAlignerColorBased m_alignerColor;
  DepthMapAlignerCurvatureBased m_alignerCurvature;
};
} // namespace

GeometryScaler::GeometryScaler(const Common::Json & /*rootNode*/,
                               const Common::Json &componentNode) {
  m_defaultGup.gup_type(MivBitstream::GupType::HVR)
      .gup_erode_threshold(
          Common::Half(componentNode.require("minForegroundConfidence").as<float>()))
      .gup_delta_threshold(componentNode.require("geometryEdgeMagnitudeTh").as<int32_t>())
      .gup_max_curvature(static_cast<uint8_t>(componentNode.require("maxCurvature").as<int32_t>()));
}

auto GeometryScaler::scale(const MivBitstream::AtlasAccessUnit &atlas,
                           const MivBitstream::GeometryUpscalingParameters &gup)
    -> Common::Depth10Frame {
  auto upscaler = DepthUpscaler{static_cast<int32_t>(gup.gup_delta_threshold()),
                                gup.gup_erode_threshold(), gup.gup_max_curvature()};

  return upscaler(atlas);
}

void GeometryScaler::inplaceScale(MivBitstream::AccessUnit &frame) const {
  for (size_t k = 0; k <= frame.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = frame.vps.vps_atlas_id(k);
    auto &atlas = frame.atlas[k];

    if (frame.vps.vps_geometry_video_present_flag(j) ||
        (frame.vps.vps_packing_information_present_flag() &&
         frame.vps.vps_packed_video_present_flag(j) &&
         frame.vps.packing_information(j).pin_geometry_present_flag())) {
      if (!atlas.attrFrame.empty() && !atlas.decGeoFrame.empty() &&
          atlas.decGeoFrame.getSize() != atlas.attrFrame.getSize()) {
        atlas.geoFrame = scale(atlas, frame.gup.value_or(m_defaultGup));
      } else {
        atlas.geoFrame = atlas.decGeoFrame;
      }
    }
  }
}
} // namespace TMIV::Decoder
