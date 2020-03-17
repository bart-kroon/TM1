/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
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

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::Decoder {
namespace {
auto getNeighborhood5() -> vector<Vec2i> {
  return {Vec2i{0, 0}, Vec2i{0, -1}, Vec2i{1, 0}, Vec2i{0, 1}, Vec2i{-1, 0}};
}

auto getNeighborhood3x3() -> vector<Vec2i> {
  return {Vec2i{0, 0}, Vec2i{0, -1}, Vec2i{1, -1}, Vec2i{1, 0},  Vec2i{1, 1},
          Vec2i{0, 1}, Vec2i{-1, 1}, Vec2i{-1, 0}, Vec2i{-1, -1}};
}

auto getNeighborhood5x5() -> vector<Vec2i> {
  return {Vec2i{0, 0},  Vec2i{0, -1}, Vec2i{1, -1},  Vec2i{1, 0},   Vec2i{1, 1},
          Vec2i{0, 1},  Vec2i{-1, 1}, Vec2i{-1, 0},  Vec2i{-1, -1}, Vec2i{0, -2},
          Vec2i{1, -2}, Vec2i{2, -2}, Vec2i{2, -1},  Vec2i{2, 0},   Vec2i{2, 1},
          Vec2i{2, 2},  Vec2i{1, 2},  Vec2i{0, 2},   Vec2i{-1, 2},  Vec2i{-2, 2},
          Vec2i{-2, 1}, Vec2i{-2, 0}, Vec2i{-2, -1}, Vec2i{-2, -2}, Vec2i{-1, -2}};
}

template <typename T>
auto sampleKernel(const Mat<T> &mat, const Vec2i &loc, const vector<Vec2i> &kernelPoints) {
  vector<T> samples(kernelPoints.size());

  auto s = samples.begin();
  for (const auto &pnt : kernelPoints) {
    auto loc_k = loc + pnt;
    *s++ = mat(loc_k[1], loc_k[0]);
  }

  return samples;
}

auto sampleKernel(const Texture444Frame &attrFrame, const Vec2i &loc,
                  const vector<Vec2i> &kernelPoints) {
  auto channels = array<vector<uint16_t>, 3>{};

  for (int d = 0; d < 3; ++d) {
    channels[d] = sampleKernel(attrFrame.getPlane(d), loc, kernelPoints);
  }

  auto samples = vector<Vec3w>(channels.front().size());

  for (size_t i = 0; i < channels.front().size(); ++i) {
    for (int d = 0; d < 3; ++d) {
      samples[i][d] = channels[d][i];
    }
  }

  return samples;
}

auto minMasked(const vector<uint16_t> &values, const vector<uint8_t> &mask) {
  auto minValue = values[0]; // original foreground value
  assert(mask[0] != 0);

  auto m = mask.begin();
  for (const auto &v : values) {
    if (*m++ == 0) {
      minValue = min(v, minValue);
    }
  }

  return minValue;
}

inline auto colorDistance(const Vec3w &a, const Vec3w &b) {
  auto dist = float(abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2]));
  return dist;
}

template <typename Range> auto meanColorDistance(const Vec3w &color, const Range &rangeOfColors) {
  int N = int(rangeOfColors.size());
  assert(N > 0U);

  float meanDistance = 0U;
  for (auto &colorInRange : rangeOfColors) {
    meanDistance += colorDistance(color, colorInRange);
  }
  meanDistance /= float(N);

  return meanDistance;
}

auto findForegroundEdges(const Mat<uint16_t> &depth) -> Mat<uint8_t> {
  using Vec4i = stack::Vec4<int>;
  auto edgeMask = Mat<uint8_t>{depth.sizes()};
  auto m_kernelPoints = getNeighborhood3x3();
  for (int i = 1; i < int(depth.height()) - 1; ++i) {
    for (int j = 1; j < int(depth.width()) - 1; ++j) {
      const auto s = sampleKernel(depth, Vec2i{j, i}, m_kernelPoints);

      auto e4 = Vec4i{s[0] - s[1], s[0] - s[3], s[0] - s[5], s[0] - s[7]};
      auto m = max(e4[0], max(e4[1], max(e4[2], e4[3])));

      edgeMask(i, j) = static_cast<uint8_t>(min(255, m));
    }
  }
  return edgeMask;
}

auto findRegionBoundaries(const Mat<uint16_t> &regionLabels) -> Mat<uint8_t> {
  auto boundaryMask = Mat<uint8_t>{regionLabels.sizes()};
  auto m_kernelPoints = getNeighborhood5();
  for (int i = 1; i < int(regionLabels.height()) - 1; ++i) {
    for (int j = 1; j < int(regionLabels.width()) - 1; ++j) {
      const auto s = sampleKernel(regionLabels, Vec2i{j, i}, m_kernelPoints);
      bool sameRegion = s[0] == s[1] && s[0] == s[2] && s[0] == s[3] && s[0] == s[4];
      boundaryMask(i, j) = sameRegion ? 0 : 255;
    }
  }
  return boundaryMask;
}

auto findForegroundEdges(const Mat<uint16_t> &depth, const Mat<uint16_t> &regionLabels)
    -> Mat<uint8_t> {
  auto edges = findForegroundEdges(depth);
  const auto bounds = findRegionBoundaries(regionLabels);
  transform(begin(edges), end(edges), begin(bounds), begin(edges),
            [](uint8_t e, uint8_t b) { return b != 0 ? uint8_t(0) : e; });
  return edges;
}

auto erodeMasked(const Mat<uint16_t> &depth, const Mat<uint8_t> &mask) -> Mat<uint16_t> {
  auto depthOut = depth;
  auto kernelPoints = getNeighborhood3x3();
  for (int i = 1; i < int(depth.height()) - 1; ++i) {
    for (int j = 1; j < int(depth.width()) - 1; ++j) {
      if (mask(i, j) != 0) {
        const auto depthSamples = sampleKernel(depth, Vec2i{j, i}, kernelPoints);
        const auto maskSamples = sampleKernel(mask, Vec2i{j, i}, kernelPoints);
        auto depthEroded = minMasked(depthSamples, maskSamples);
        depthOut(i, j) = depthEroded;
      }
    }
  }
  return depthOut;
}

class DepthMapAlignerColorBased {
public:
  DepthMapAlignerColorBased(int depthEdgeMagnitudeTh, float minForegroundConfidence)
      : m_depthEdgeMagnitudeTh{depthEdgeMagnitudeTh},
        m_minForegroundConfidence{minForegroundConfidence}, m_kernelPoints{getNeighborhood5x5()} {}

  auto colorConfidence(const vector<uint16_t> &depthValues, const vector<Vec3w> &colorValues,
                       const vector<uint8_t> &edgeMagnitudes) -> float {
    const auto N = depthValues.size();
    assert(N == colorValues.size());
    assert(N == edgeMagnitudes.size());

    const int depthCentral = depthValues[0];
    const int depthLow = depthCentral - m_depthEdgeMagnitudeTh;
    const int depthHigh = depthCentral + m_depthEdgeMagnitudeTh;
    const auto colorCentral = colorValues[0];

    // split colors samples in kernel in foreground and background
    // exclude the (uncertain) depth edges and pixels beyond foreground
    vector<Vec3w> colorsFG;
    vector<Vec3w> colorsBG;
    for (auto i = 1U; i < N; ++i) {
      if (edgeMagnitudes[i] < m_depthEdgeMagnitudeTh && depthValues[i] < depthHigh) {
        if (depthValues[i] > depthLow) {
          colorsFG.push_back(colorValues[i]);
        } else {
          colorsBG.push_back(colorValues[i]);
        }
      }
    }

    // make the groups of equal size
    int groupSize = int(min(colorsFG.size(), colorsBG.size()));

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

  auto colorConfidenceAt(const Texture444Frame &attrFrame, const Mat<uint16_t> &depth,
                         const Mat<uint8_t> &edgeMagnitudes, const Vec2i &loc) -> float {
    auto depths = sampleKernel(depth, loc, m_kernelPoints);
    auto colors = sampleKernel(attrFrame, loc, m_kernelPoints);
    auto edges = sampleKernel(edgeMagnitudes, loc, m_kernelPoints);

    return colorConfidence(depths, colors, edges);
  }

  auto operator()(const Texture444Frame &attrFrame, const Mat<uint16_t> &depth,
                  const Mat<uint8_t> &edgeMagnitudes) -> Mat<uint16_t> {
    const int numIterations = 1;
    auto depthIter = depth;
    for (int iter = 0; iter < numIterations; iter++) {
      auto markers = Mat<uint8_t>{depth.sizes()};
      for (int i = m_B; i < int(depth.height()) - m_B; ++i) {
        for (int j = m_B; j < int(depth.width()) - m_B; ++j) {
          if (edgeMagnitudes(i, j) >= m_depthEdgeMagnitudeTh) {
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
  int m_depthEdgeMagnitudeTh;
  float m_minForegroundConfidence;
  vector<Vec2i> m_kernelPoints;
  int m_B = 2;
};

class DepthMapAlignerCurvatureBased {
public:
  DepthMapAlignerCurvatureBased(int depthEdgeMagnitudeTh, int maxCurvature)
      : m_depthEdgeMagnitudeTh(depthEdgeMagnitudeTh),
        m_maxCurvature(maxCurvature), m_kernelPoints{getNeighborhood3x3()} {}

  auto curvature(const vector<uint16_t> &depthValues) -> int {
    const int depthCentral = depthValues[0];
    const int depthLow = depthCentral - m_depthEdgeMagnitudeTh;

    int depthCurvature3x3 = 0;
    for (auto i = 1U; i < depthValues.size(); ++i) {
      if (int(depthValues[i]) < depthLow) {
        depthCurvature3x3++;
      }
    }

    return depthCurvature3x3;
  }

  auto curvatureAt(const Mat<uint16_t> &depth, const Vec2i &loc) -> int {
    auto depths = sampleKernel(depth, loc, m_kernelPoints);

    return curvature(depths);
  }

  auto operator()(const Mat<uint16_t> &depth, const Mat<uint8_t> &edgeMagnitudes) -> Mat<uint16_t> {
    auto depthOut = depth;
    auto markers = Mat<uint8_t>{depth.sizes()};

    for (int i = m_B; i < int(depth.height()) - m_B; ++i) {
      for (int j = m_B; j < int(depth.width()) - m_B; ++j) {
        if (edgeMagnitudes(i, j) >= m_depthEdgeMagnitudeTh) {
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
  int m_depthEdgeMagnitudeTh = 11;
  int m_maxCurvature = 6;
  vector<Vec2i> m_kernelPoints;
  int m_B = 1;
};

auto upscaleNearest(const Mat<uint16_t> &input, Vec2i outputSize) -> Mat<uint16_t> {
  const auto inputSize = Vec2i{int(input.width()), int(input.height())};
  auto output = Mat<uint16_t>({size_t(outputSize.y()), size_t(outputSize.x())});

  for (int yo = 0; yo < outputSize.y(); ++yo) {
    for (int xo = 0; xo < outputSize.x(); ++xo) {
      const auto xi = xo * inputSize.x() / outputSize.x();
      const auto yi = yo * inputSize.y() / outputSize.y();
      output(yo, xo) = input(yi, xi);
    }
  }

  return output;
}

class DepthUpscaler {
public:
  DepthUpscaler(int depthEdgeMagnitudeTh, float minForegroundConfidence, int maxCurvature)
      : m_alignerColor(depthEdgeMagnitudeTh, minForegroundConfidence),
        m_alignerCurvature(depthEdgeMagnitudeTh, maxCurvature) {}

  auto operator()(const AtlasAccessUnit &atlas) -> Depth10Frame {
    auto geoFrame = Depth10Frame{atlas.frameSize().x(), atlas.frameSize().y()};

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

GeometryScaler::GeometryScaler(const Json & /*rootNode*/, const Json &componentNode) {
  m_depthEdgeMagnitudeTh = componentNode.require("depthEdgeMagnitudeTh").asInt();
  m_maxCurvature = componentNode.require("maxCurvature").asInt();
  m_minForegroundConfidence = componentNode.require("minForegroundConfidence").asFloat();
}

auto GeometryScaler::scale(const AtlasAccessUnit &atlas) const -> Depth10Frame {
  auto upscaler = DepthUpscaler{m_depthEdgeMagnitudeTh, m_minForegroundConfidence, m_maxCurvature};

  return upscaler(atlas);
}

void GeometryScaler::inplaceScale(MivBitstream::AccessUnit &frame) const {
  for (auto &atlas : frame.atlas) {
    if (!atlas.attrFrame.empty() && atlas.decGeoFrame.getSize() != atlas.attrFrame.getSize()) {
      atlas.geoFrame = scale(atlas);
    } else {
      atlas.geoFrame = atlas.decGeoFrame;
    }
  }
}
} // namespace TMIV::Decoder