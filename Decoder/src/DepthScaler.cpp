/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

#include <TMIV/Decoder/DepthScaler.h>

#include <functional>
#include <iostream>

namespace TMIV::Decoder {

struct Size {
  int width;
  int height;
};

template <class MatType> auto getSize(const MatType &mat) {
  return Size{int(mat.width()), int(mat.height())};
}

auto operator*(const Size &size, int value) -> Size {
  return Size{size.width * value, size.height * value};
}

inline auto operator==(const Size &a, const Size &b) -> bool {
  return a.width == b.width && a.height == b.height;
}

template <class T> auto createMat(const Size &size) {
  TMIV::Common::heap::Matrix<T> mat;
  mat.resize(size.height, size.width);
  std::fill(mat.begin(), mat.end(), static_cast<T>(0));
  return mat;
}

template <class T>
void copyMat(const TMIV::Common::heap::Matrix<T> &from, TMIV::Common::heap::Matrix<T> &to) {
  if (!(getSize(from) == getSize(to))) {
    to.resize(from.height(), from.width());
  }

  std::copy(from.begin(), from.end(), to.begin());
}

template <class T> auto cloneMat(const TMIV::Common::heap::Matrix<T> &from) {
  auto to = createMat<T>(getSize(from));
  copyMat(from, to);
  return to;
}

template <class T1, class T2> void setTo(Mat_<T1> &mat, const T2 &newValue) {
  static_assert(std::is_convertible<T2, T1>::value);
  for (auto &value : mat) {
    value = newValue;
  }
}

template <class T1, class T2> auto countValue(const Mat_<T1> &mat, const T2 &cmpValue) {
  static_assert(std::is_convertible<T2, T1>::value);
  return std::count(mat.begin(), mat.end(), cmpValue);
}

template <class T, class F>
auto matCombine(const Mat_<T> &matA, const Mat_<T> &matB, F func) -> Mat_<T> {
  auto size = getSize(matA);
  assert(getSize(matB) == size);
  auto matR = createMat<T>(size);

  auto a = matA.begin();
  auto b = matB.begin();
  for (auto &combinedValue : matR) {
    combinedValue = static_cast<T>(func(*a++, *b++));
  }

  return matR;
}

auto getNeighborhood2x2() -> std::vector<Vec2i> {
  return {Vec2i{0, 0}, Vec2i{1, 0}, Vec2i{0, 1}, Vec2i{1, 1}};
}

auto getNeighborhood5() -> std::vector<Vec2i> {
  return {Vec2i{0, 0}, Vec2i{0, -1}, Vec2i{1, 0}, Vec2i{0, 1}, Vec2i{-1, 0}};
}

auto getNeighborhood3x3() -> std::vector<Vec2i> {
  return {Vec2i{0, 0}, Vec2i{0, -1}, Vec2i{1, -1}, Vec2i{1, 0},  Vec2i{1, 1},
          Vec2i{0, 1}, Vec2i{-1, 1}, Vec2i{-1, 0}, Vec2i{-1, -1}};
}

auto getNeighborhood5x5() -> std::vector<Vec2i> {
  return {Vec2i{0, 0},  Vec2i{0, -1}, Vec2i{1, -1},  Vec2i{1, 0},   Vec2i{1, 1},
          Vec2i{0, 1},  Vec2i{-1, 1}, Vec2i{-1, 0},  Vec2i{-1, -1}, Vec2i{0, -2},
          Vec2i{1, -2}, Vec2i{2, -2}, Vec2i{2, -1},  Vec2i{2, 0},   Vec2i{2, 1},
          Vec2i{2, 2},  Vec2i{1, 2},  Vec2i{0, 2},   Vec2i{-1, 2},  Vec2i{-2, 2},
          Vec2i{-2, 1}, Vec2i{-2, 0}, Vec2i{-2, -1}, Vec2i{-2, -2}, Vec2i{-1, -2}};
}

template <class MatType>
auto sampleKernel(const MatType &mat, const Vec2i &loc, const std::vector<Vec2i> &kernelPoints) {
  std::vector<typename MatType::value_type> samples(kernelPoints.size());

  auto s = samples.begin();
  for (const auto &pnt : kernelPoints) {
    auto loc_k = loc + pnt;
    *s++ = mat(loc_k[1], loc_k[0]);
  }

  return samples;
}

inline auto onImage(const Size &sz, const Vec2i &pt) -> bool {
  return pt[0] >= 0 && pt[0] < sz.width && pt[1] >= 0 && pt[1] < sz.height;
}

auto minMasked(const std::vector<ushort> &values, const std::vector<uchar> &mask) {
  auto minValue = values[0]; // original foreground value
  assert(mask[0] != 0);

  auto m = mask.begin();
  for (const auto &v : values) {
    if (*m++ == 0) {
      minValue = std::min(v, minValue);
    }
  }

  return minValue;
}

auto minMasked(const std::vector<ushort> &values, const std::vector<ushort> &labels,
               const std::vector<uchar> &mask) {
  auto minValue = values[0]; // original foreground value
  auto minLabel = labels[0];
  assert(mask[0] != 0);

  auto m = mask.begin();
  auto l = labels.begin();
  for (const auto &v : values) {
    if (*m == 0) {
      if (v < minValue) {
        minValue = v;
        minLabel = *l;
      }
    }
    m++;
    l++;
  }

  return std::pair{minValue, minLabel};
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

auto findForegroundEdges(const Mat1w &depth) -> Mat1b {
  using Vec4i = TMIV::Common::stack::Vec4<int>;
  auto size = getSize(depth);
  Mat1b edgeMask = createMat<unsigned char>(size);
  auto kernelPoints = getNeighborhood3x3();
  for (int i = 1; i < size.height - 1; ++i) {
    for (int j = 1; j < size.width - 1; ++j) {
      const auto s = sampleKernel(depth, Vec2i{j, i}, kernelPoints);

      auto e4 = Vec4i{s[0] - s[1], s[0] - s[3], s[0] - s[5], s[0] - s[7]};
      auto m = std::max(e4[0], std::max(e4[1], std::max(e4[2], e4[3])));

      edgeMask(i, j) = static_cast<unsigned char>(std::min(255, m));
    }
  }
  return edgeMask;
}

auto findRegionBoundaries(const Mat1w &regionLabels) -> Mat1b {
  auto size = getSize(regionLabels);
  Mat1b boundaryMask = createMat<unsigned char>(size);
  auto kernelPoints = getNeighborhood5();
  for (int i = 1; i < size.height - 1; ++i) {
    for (int j = 1; j < size.width - 1; ++j) {
      const auto s = sampleKernel(regionLabels, Vec2i{j, i}, kernelPoints);
      bool sameRegion = s[0] == s[1] && s[0] == s[2] && s[0] == s[3] && s[0] == s[4];
      boundaryMask(i, j) = sameRegion ? 0 : 255;
    }
  }
  return boundaryMask;
}

auto findForegroundEdges(const Mat1w &depth, const Mat1w &regionLabels) -> Mat1b {
  auto edges = findForegroundEdges(depth);
  auto bounds = findRegionBoundaries(regionLabels);
  edges = matCombine(edges, bounds, [](uchar e, uchar b) { return b != 0 ? 0U : e; });
  return edges;
}

auto erodeMasked(const Mat1w &depth, const Mat1b &mask) -> Mat1w {
  auto depthOut = cloneMat(depth);
  auto size = getSize(depth);
  auto kernelPoints = getNeighborhood3x3();
  for (int i = 1; i < size.height - 1; ++i) {
    for (int j = 1; j < size.width - 1; ++j) {
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

DepthMapAlignerColorBased::DepthMapAlignerColorBased(int depthEdgeMagnitudeTh,
                                                     float minForegroundConfidence)
    : m_depthEdgeMagnitudeTh(depthEdgeMagnitudeTh),
      m_minForegroundConfidence(minForegroundConfidence) {
  m_kernelPoints = getNeighborhood5x5();
}

auto DepthMapAlignerColorBased::colorConfidence(const std::vector<ushort> &depthValues,
                                                const std::vector<Vec3w> &colorValues,
                                                const std::vector<uchar> &edgeMagnitudes) -> float {
  const auto N = depthValues.size();
  assert(N == colorValues.size());
  assert(N == edgeMagnitudes.size());

  const int depthCentral = depthValues[0];
  const int depthLow = depthCentral - m_depthEdgeMagnitudeTh;
  const int depthHigh = depthCentral + m_depthEdgeMagnitudeTh;
  const auto colorCentral = colorValues[0];

  // split colors samples in kernel in foreground and background
  // exclude the (uncertain) depth edges and pixels beyond foreground
  std::vector<Vec3w> colorsFG;
  std::vector<Vec3w> colorsBG;
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
  int groupSize = int(std::min(colorsFG.size(), colorsBG.size()));

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

auto DepthMapAlignerColorBased::colorConfidenceAt(const Mat3w &yuv, const Mat1w &depth,
                                                  const Mat1b &edgeMagnitudes, const Vec2i &loc)
    -> float {
  auto depths = sampleKernel(depth, loc, m_kernelPoints);
  auto colors = sampleKernel(yuv, loc, m_kernelPoints);
  auto edges = sampleKernel(edgeMagnitudes, loc, m_kernelPoints);

  return colorConfidence(depths, colors, edges);
}

auto DepthMapAlignerColorBased::operator()(const Mat3w &yuv, const Mat1w &depth,
                                           const Mat1b &edgeMagnitudes) -> Mat1w {
  const int numIterations = 1;
  auto size = getSize(depth);
  auto depthIter = cloneMat(depth);
  for (int iter = 0; iter < numIterations; iter++) {
    m_markers = createMat<uchar>(size);
    for (int i = m_B; i < size.height - m_B; ++i) {
      for (int j = m_B; j < size.width - m_B; ++j) {
        if (edgeMagnitudes(i, j) >= m_depthEdgeMagnitudeTh) {
          auto foregroundConfidence = colorConfidenceAt(yuv, depthIter, edgeMagnitudes, {j, i});
          if (foregroundConfidence < m_minForegroundConfidence) {
            m_markers(i, j) = 255;
          }
        }
      }
    }
    depthIter = erodeMasked(depthIter, m_markers);
  }
  return depthIter;
}

DepthMapAlignerCurvatureBased::DepthMapAlignerCurvatureBased(int depthEdgeMagnitudeTh,
                                                             int maxCurvature)
    : m_depthEdgeMagnitudeTh(depthEdgeMagnitudeTh), m_maxCurvature(maxCurvature) {
  kernelPoints = getNeighborhood3x3();
}

auto DepthMapAlignerCurvatureBased::curvature(const std::vector<ushort> &depthValues) -> int {
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

auto DepthMapAlignerCurvatureBased::curvatureAt(const Mat1w &depth, const Vec2i &loc) -> int {
  auto depths = sampleKernel(depth, loc, kernelPoints);

  return curvature(depths);
}

auto DepthMapAlignerCurvatureBased::operator()(const Mat1w &depth, const Mat1b &edgeMagnitudes)
    -> Mat1w {
  auto size = getSize(depth);
  auto depthOut = cloneMat(depth);

  m_markers = createMat<uchar>(size);
  for (int i = m_B; i < size.height - m_B; ++i) {
    for (int j = m_B; j < size.width - m_B; ++j) {
      if (edgeMagnitudes(i, j) >= m_depthEdgeMagnitudeTh) {
        auto curvature = curvatureAt(depth, {j, i});
        if (curvature >= m_maxCurvature) {
          m_markers(i, j) = 255;
        }
      }
    }
  }

  depthOut = erodeMasked(depth, m_markers);

  return depthOut;
}

auto upscaleNearest(const Mat1w &depth) -> Mat1w {
  auto size = getSize(depth);
  auto sizeU2 = size * 2;

  Mat1w depthU2 = createMat<ushort>(sizeU2);

  for (int i = 0; i < size.height; ++i) {
    for (int j = 0; j < size.width; ++j) {
      auto d = depth(i, j);
      depthU2(2 * i, 2 * j) = d;
      depthU2(2 * i, 2 * j + 1) = d;
      depthU2(2 * i + 1, 2 * j) = d;
      depthU2(2 * i + 1, 2 * j + 1) = d;
    }
  }

  return depthU2;
}

auto yuv420_To_Mat3w(const TMIV::Common::Frame<TMIV::Common::YUV420P10> &frame) -> Mat3w {
  const auto &Y = frame.getPlane(0);
  const auto &U = frame.getPlane(1);
  const auto &V = frame.getPlane(2);

  auto size = getSize(Y);
  auto yuv = createMat<Vec3w>(size);
  for (int i = 0; i < size.height; ++i) {
    for (int j = 0; j < size.width; ++j) {
      yuv(i, j) = {Y(i, j), U(i / 2, j / 2), V(i / 2, j / 2)};
    }
  }

  return yuv;
}

DepthUpscaler::DepthUpscaler(int depthEdgeMagnitudeTh, float minForegroundConfidence,
                             int maxCurvature)
    : m_alignerColor(depthEdgeMagnitudeTh, minForegroundConfidence),
      m_alignerCurvature(depthEdgeMagnitudeTh, maxCurvature) {}

auto DepthUpscaler::operator()(const Mat1w &depthD2, const Mat3w &yuv) -> Mat1w {
  m_depthUpscaled = upscaleNearest(depthD2);
  assert(getSize(m_depthUpscaled) == getSize(yuv));

  // Erode based on color alignment
  m_edgeMagnitudes1 = findForegroundEdges(m_depthUpscaled);
  m_depthColorAligned = m_alignerColor(yuv, m_depthUpscaled, m_edgeMagnitudes1);

  // Erode based on (fg) curvature
  m_edgeMagnitudes2 = findForegroundEdges(m_depthColorAligned);
  m_depthCurvatureAligned = m_alignerCurvature(m_depthColorAligned, m_edgeMagnitudes2);

  return m_depthCurvatureAligned;
}

auto DepthUpscaler::operator()(const Mat1w &depthD2, const Mat1w &regionsD2, const Mat3w &yuv)
    -> std::pair<Mat1w, Mat1w> {
  m_depthUpscaled = upscaleNearest(depthD2);
  m_regionsUpscaled = upscaleNearest(regionsD2);
  assert(getSize(m_depthUpscaled) == getSize(yuv));
  assert(getSize(m_regionsUpscaled) == getSize(yuv));

  // Erode based on color alignment
  m_edgeMagnitudes1 = findForegroundEdges(m_depthUpscaled, m_regionsUpscaled);
  m_depthColorAligned = m_alignerColor(yuv, m_depthUpscaled, m_edgeMagnitudes1);

  // Erode based on (fg) curvature
  m_edgeMagnitudes2 = findForegroundEdges(m_depthColorAligned, m_regionsUpscaled);
  m_depthCurvatureAligned = m_alignerCurvature(m_depthColorAligned, m_edgeMagnitudes2);

  return std::pair(m_depthCurvatureAligned, m_regionsUpscaled);
}

DepthUpscalerAtlas::DepthUpscalerAtlas(const Common::Json & /*rootNode*/,
                                       const Common::Json &componentNode) {
  m_depthEdgeMagnitudeTh = componentNode.require("depthEdgeMagnitudeTh").asInt();
  m_maxCurvature = componentNode.require("maxCurvature").asInt();
  m_minForegroundConfidence = componentNode.require("minForegroundConfidence").asFloat();
}

auto DepthUpscalerAtlas::upsampleDepthAndOccupancyMapMVD(
    const TMIV::Common::MVD10Frame &atlas, const TMIV::Common::PatchIdMapList &maps) const
    -> std::pair<TMIV::Common::MVD10Frame, TMIV::Common::PatchIdMapList> {
  assert(atlas.size() == maps.size());

  TMIV::Common::MVD10Frame atlasOut(atlas.size());
  TMIV::Common::PatchIdMapList mapsOut(maps.size());

  int w = atlas[0].first.getWidth();
  int h = atlas[0].first.getHeight();

  for (auto &frame : atlasOut) {
    frame.second.resize(w, h);
  }

  for (auto &frame : mapsOut) {
    frame.resize(w, h);
  }

  DepthUpscaler upscaler(m_depthEdgeMagnitudeTh, m_minForegroundConfidence, m_maxCurvature);

  for (auto i = 0U; i < atlas.size(); ++i) {
    const auto yuv = yuv420_To_Mat3w(atlas[i].first);
    const auto &depthD2 = atlas[i].second.getPlane(0);
    const auto &labelsD2 = maps[i].getPlane(0);

    atlasOut[i].first = atlas[i].first;
    auto &depth = atlasOut[i].second.getPlane(0);
    auto &labels = mapsOut[i].getPlane(0);

    std::tie(depth, labels) = upscaler(depthD2, labelsD2, yuv);
  }

  return std::pair{atlasOut, mapsOut};
}

} // namespace TMIV::Decoder