/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#include <TMIV/Pruner/HierarchicalPruner.h>

#include "IncrementalSynthesizer.h"
#include "LumaStdDev.h"
#include "PrunedMesh.h"
#include <TMIV/Common/Graph.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <TMIV/Renderer/reprojectPoints.h>

#include <algorithm>
#include <cmath>
#include <future>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <optional>

namespace TMIV::Pruner {
namespace {
auto convertYuvPixelToRgbPixel(const Common::Vec3f &yuv, const float maxIntensity = 255.F)
    -> Common::Vec3f {
  const float invertedMaxIntensity = 1.F / maxIntensity;
  return {1.164F * (yuv[0] - 16.F * invertedMaxIntensity) +
              2.018F * (yuv[1] - 128.F * invertedMaxIntensity),
          1.164F * (yuv[0] - 16.F * invertedMaxIntensity) -
              0.813F * (yuv[2] - 128.F * invertedMaxIntensity) -
              0.391F * (yuv[1] - 128.F * invertedMaxIntensity),
          1.164F * (yuv[0] - 16.F * invertedMaxIntensity) +
              1.596F * (yuv[2] - 128.F * invertedMaxIntensity)};
}

auto createRgbImageFromYuvImage(const Common::Mat<Common::Vec3f> &yuvImage) {
  Common::Mat<Common::Vec3f> rgbImage{yuvImage};
  std::transform(rgbImage.begin(), rgbImage.end(), rgbImage.begin(),
                 [](const auto &yuv_pixel) { return convertYuvPixelToRgbPixel(yuv_pixel); });
  return rgbImage;
}

auto iterativeReweightedLeastSquaresOnNonPrunedPixels(
    const std::vector<size_t> &nonPrunedPixIndices, const Common::Mat<Common::Vec3f> &referenceRGB,
    const Common::Mat<Common::Vec3f> &synthesizedRGB, const std::vector<float> &weights,
    const size_t colorChannel, const double eps) {
  const size_t numPixels = nonPrunedPixIndices.size();
  Common::Mat<double> A({numPixels, 4});
  Common::Mat<double> b({numPixels, 1});
  Common::Mat<double> x({4, 1});
  Common::Mat<double> A_t;
  Common::Mat<double> A_pseudo;
  for (size_t i = 0; i < numPixels; ++i) {
    size_t pixelIdx = nonPrunedPixIndices[i];
    const auto rR = static_cast<double>(referenceRGB[pixelIdx][0]);
    const auto rG = static_cast<double>(referenceRGB[pixelIdx][1]);
    const auto rB = static_cast<double>(referenceRGB[pixelIdx][2]);
    const auto s = static_cast<double>(synthesizedRGB[pixelIdx][colorChannel]);

    const auto w = static_cast<double>(weights[i]);

    A(i, 0) = w * rR;
    A(i, 1) = w * rG;
    A(i, 2) = w * rB;
    A(i, 3) = w * 1.F;
    b(i, 0) = w * s;
  }

  transpose(A, A_t);
  transquare(A, A_pseudo);
  A_pseudo += eps * Common::Mat<double>::eye(A_pseudo.sizes());
  mldivide(A_pseudo, A_t * b, x);
  return x;
}

auto computeWeightedRgbSum(const Common::Vec3f &referenceRGB, const Common::Mat<double> &x)
    -> double {
  auto rR = static_cast<double>(referenceRGB[0]);
  auto rG = static_cast<double>(referenceRGB[1]);
  auto rB = static_cast<double>(referenceRGB[2]);

  return x(0, 0) * rR + x(1, 0) * rG + x(2, 0) * rB + x(3, 0);
}

auto computeNonPrunedPixelIndices(const Common::Mat<Common::Vec3f> &synthesizedYUV,
                                  const Common::Mat<uint8_t> &prunedMask) -> std::vector<size_t> {
  std::vector<size_t> nonPrunedPixIndices;
  for (size_t i = 0; i < prunedMask.size(); ++i) {
    if (synthesizedYUV[i][0] < 0.1F && synthesizedYUV[i][1] < 0.1F && synthesizedYUV[i][2] < 0.1F) {
      continue;
    }
    if (prunedMask[i] > 0) {
      nonPrunedPixIndices.push_back(i);
    }
  }
  return nonPrunedPixIndices;
}
} // namespace
const auto depthErrorEps = 1E-4F;

class HierarchicalPruner::Impl {
private:
  const float m_maxDepthError{};
  const float m_maxLumaError{};
  const float m_maxStretching{};
  const int32_t m_erode{};
  const int32_t m_dilate{};
  const int32_t m_maxBasicViewsPerGraph{};
  const bool m_enable2ndPassPruner{};
  const int32_t m_sampleSize{};
  int32_t m_reviveRatio{100};
  const float m_maxColorError{};
  const Renderer::AccumulatingPixel<Common::Vec3f> m_config;
  std::optional<float> m_lumaStdDev{};
  PrunerParams m_params;
  std::vector<std::unique_ptr<IncrementalSynthesizer>> m_synthesizers;
  std::vector<size_t> m_clusterIds;
  struct Cluster {
    std::vector<size_t> basicViewId;
    std::vector<size_t> additionalViewId;
    std::vector<size_t> pruningOrder;
  };
  std::vector<Cluster> m_clusters;
  Common::FrameList<uint8_t> m_masks;
  Common::FrameList<uint8_t> m_status;

public:
  explicit Impl(const Common::Json &nodeConfig)
      : m_maxDepthError{nodeConfig.require("maxDepthError").as<float>()}
      , m_maxLumaError{nodeConfig.require("maxLumaError").as<float>()}
      , m_maxStretching{nodeConfig.require("maxStretching").as<float>()}
      , m_erode{nodeConfig.require("erode").as<int32_t>()}
      , m_dilate{nodeConfig.require("dilate").as<int32_t>()}
      , m_maxBasicViewsPerGraph{nodeConfig.require("maxBasicViewsPerGraph").as<int32_t>()}
      , m_enable2ndPassPruner{nodeConfig.require("enable2ndPassPruner").as<bool>()}
      , m_sampleSize{nodeConfig.require("sampleSize").as<int32_t>()}
      , m_maxColorError{nodeConfig.require("maxColorError").as<float>()}
      , m_config{nodeConfig.require("rayAngleParameter").as<float>(),
                 nodeConfig.require("depthParameter").as<float>(),
                 nodeConfig.require("stretchingParameter").as<float>(), m_maxStretching} {}

  static void assignAdditionalViews(const Common::Mat<float> &overlap,
                                    const MivBitstream::ViewParamsList &viewParamsList,
                                    size_t numClusters, std::vector<size_t> &clusterIds) {
    const auto N = viewParamsList.size();
    auto numViewsPerCluster = std::vector<size_t>(numClusters, 0);
    for (size_t i = 0; i < N; ++i) {
      if (viewParamsList[i].isBasicView) {
        const auto c = clusterIds[i];
        ++numViewsPerCluster[c];
      }
    }
    for (;;) {
      auto minCount = N;
      auto maxOverlap = 0.F;
      size_t basicViewId = 0;
      size_t additionalViewId = 0;

      for (size_t i = 0; i < N; ++i) {
        const auto c_i = clusterIds[i];
        if (viewParamsList[i].isBasicView) {
          for (size_t j = 0; j < N; ++j) {
            if (!viewParamsList[j].isBasicView && clusterIds[j] == numClusters) {
              if (minCount > numViewsPerCluster[c_i] ||
                  (minCount == numViewsPerCluster[c_i] && maxOverlap < overlap(i, j))) {
                minCount = numViewsPerCluster[c_i];
                maxOverlap = overlap(i, j);
                basicViewId = i;
                additionalViewId = j;
              }
            }
          }
        }
      }
      if (minCount == N) {
        break;
      }
      const auto c = clusterIds[basicViewId];
      ++numViewsPerCluster[c];
      clusterIds[additionalViewId] = c;
    }
  }

  static auto scoreClustering(const Common::Mat<float> &overlap,
                              const std::vector<size_t> &clusterIds) -> double {
    auto score = 0.;
    const auto N = overlap.height();

    for (size_t i = 0; i < N; ++i) {
      for (size_t j = i + 1; j < N; ++j) {
        if (clusterIds[i] == clusterIds[j]) {
          score += overlap(i, j);
        }
      }
    }
    return score;
  }

  [[nodiscard]] auto exhaustiveSearch(const Common::Mat<float> &overlap,
                                      const MivBitstream::ViewParamsList &viewParamsList) const {
    auto basicViewIds = std::vector<size_t>{};
    auto haveAdditionalViews = false;
    for (size_t i = 0; i < viewParamsList.size(); ++i) {
      if (viewParamsList[i].isBasicView) {
        basicViewIds.push_back(i);
      } else {
        haveAdditionalViews = true;
      }
    }

    if (!haveAdditionalViews) {
      // NOTE(BK): Avoid exhaustive search on R17 SB
      return std::vector<size_t>(viewParamsList.size(), 0);
    }

    const size_t maxBasicViews = m_maxBasicViewsPerGraph;
    const auto numClusters = (basicViewIds.size() + maxBasicViews - 1) / maxBasicViews;
    POSTCONDITION(numClusters >= 1);

    size_t numPermutations = 1;
    for (size_t i = 0; i < basicViewIds.size(); ++i) {
      numPermutations *= numClusters;
    }

    auto clusterIds = std::vector<size_t>(viewParamsList.size());
    auto numBasicViewsPerCluster = std::vector<size_t>(numClusters);

    auto bestScore = 0.;
    auto bestClusterIds = std::vector<size_t>{};

    for (size_t p = 0; p < numPermutations; ++p) {
      auto q = p;
      std::fill(clusterIds.begin(), clusterIds.end(), numClusters);
      std::fill(numBasicViewsPerCluster.begin(), numBasicViewsPerCluster.end(), 0);
      auto valid = true;
      for (auto i : basicViewIds) {
        const auto j = q % numClusters;
        q /= numClusters;
        clusterIds[i] = j;
        if (++numBasicViewsPerCluster[j] > maxBasicViews) {
          valid = false;
          break;
        }
      }
      if (valid) {
        assignAdditionalViews(overlap, viewParamsList, numClusters, clusterIds);
        const auto score = scoreClustering(overlap, clusterIds);

        if (bestScore < score) {
          bestScore = score;
          bestClusterIds = clusterIds;
        }
      }
    }

    POSTCONDITION(!bestClusterIds.empty());
    return bestClusterIds;
  }

  void clusterViews(const Common::Mat<float> &overlap,
                    const MivBitstream::ViewParamsList &viewParamsList) {
    const auto clusterIds = exhaustiveSearch(overlap, viewParamsList);

    m_clusters = std::vector<Cluster>(1 + *max_element(clusterIds.cbegin(), clusterIds.cend()));
    for (size_t i = 0; i < clusterIds.size(); ++i) {
      if (viewParamsList[i].isBasicView) {
        m_clusters[clusterIds[i]].basicViewId.push_back(i);
      } else {
        m_clusters[clusterIds[i]].additionalViewId.push_back(i);
      }
    }
  }

  void computePruningOrder(const Common::Mat<float> &overlappingMatrix) {
    for (auto &cluster : m_clusters) {
      auto processedList = cluster.basicViewId;
      auto pendingList = cluster.additionalViewId;
      cluster.pruningOrder.clear();

      while (!pendingList.empty()) {
        float worseOverlapping = std::numeric_limits<float>::max();
        size_t bestPendingNodeId = 0;

        for (auto pendingNodeId : pendingList) {
          for (auto processNodeId : processedList) {
            float overlapping = overlappingMatrix(processNodeId, pendingNodeId);

            if (overlapping < worseOverlapping) {
              bestPendingNodeId = pendingNodeId;
              worseOverlapping = overlapping;
            }
          }
        }

        processedList.emplace_back(bestPendingNodeId);

        auto iter = find(pendingList.begin(), pendingList.end(), bestPendingNodeId);
        pendingList.erase(iter);

        cluster.pruningOrder.push_back(bestPendingNodeId);
      }
    }
  }

  void printClusters(const MivBitstream::ViewParamsList &vpl) const {
    std::cout << "Pruning graph:\n";
    for (const auto &cluster : m_clusters) {
      std::cout << "  (";
      for (auto i : cluster.basicViewId) {
        std::cout << ' ' << vpl[i].name;
      }
      std::cout << " )";
      for (auto i : cluster.pruningOrder) {
        std::cout << " <- " << vpl[i].name;
      }
      std::cout << '\n';
    }
  }

  auto prepareSequence(const PrunerParams &params) -> std::vector<MivBitstream::PruningParents> {
    auto pruningParents = std::vector<MivBitstream::PruningParents>(params.viewParamsList.size());

    Renderer::ProjectionHelperList cameraHelperList{params.viewParamsList};

    // Create clusters and pruning order
    auto overlappingMatrix = computeOverlappingMatrix(cameraHelperList);
    clusterViews(overlappingMatrix, params.viewParamsList);
    computePruningOrder(overlappingMatrix);
    printClusters(params.viewParamsList);

    // Pruning graph
    Common::Graph::SparseDirectedAcyclicGraph<float> pruningGraph(params.viewParamsList.size());

    for (auto &cluster : m_clusters) {
      if (!cluster.pruningOrder.empty()) {
        for (auto i : cluster.basicViewId) {
          pruningGraph.connect(cluster.pruningOrder.front(), i, 1.F);
        }
        for (size_t i = 1; i < cluster.pruningOrder.size(); ++i) {
          pruningGraph.connect(cluster.pruningOrder[i], cluster.pruningOrder[i - 1], 1.F);
        }
      }
    }

    // Pruning mask
    for (size_t viewIdx = 0; viewIdx < params.viewParamsList.size(); viewIdx++) {
      const auto &neighbourhood = pruningGraph.getNeighbourhood(viewIdx);

      if (neighbourhood.empty()) {
        pruningParents[viewIdx] = MivBitstream::PruningParents{};
      } else {
        std::vector<uint16_t> parentIdList;

        parentIdList.reserve(neighbourhood.size());

        for (const auto &link : neighbourhood) {
          parentIdList.emplace_back(static_cast<uint16_t>(link.id()));
        }

        pruningParents[viewIdx] = MivBitstream::PruningParents{std::move(parentIdList)};
      }
    }
    m_params.depthLowQualityFlag = params.depthLowQualityFlag;
    m_params.sampleBudget = params.sampleBudget;
    return pruningParents;
  }

  auto prune(const MivBitstream::ViewParamsList &viewParamsList, const Common::DeepFrameList &views)
      -> Common::FrameList<uint8_t> {
    m_params.viewParamsList = viewParamsList;

    bool isItFirstFrame = false;
    if (!m_lumaStdDev.has_value()) {
      isItFirstFrame = true;
      const auto lumaStdDev =
          calculateLumaStdDev(views, m_params.viewParamsList, m_config, m_maxDepthError);
      if (lumaStdDev.has_value() && *lumaStdDev != 0.F) {
        m_lumaStdDev.emplace(lumaStdDev.value());
      } else {
        m_lumaStdDev.emplace(1.0F); // to preserve initial maxLumaError value set in config .json
      }
    }

    prepareFrame(views);
    auto nonPrunedArea = pruneFrame(views);

    if (isItFirstFrame) {
      analyzeFillAndPruneAgain(views, nonPrunedArea, 80);
    }

    return std::move(m_masks);
  }

private:
  void analyzeFillAndPruneAgain(const Common::DeepFrameList &views, int32_t nonPrunedArea,
                                int32_t percentageRatio) {
    const float A = 0.5F / (1.F - m_lumaStdDev.value());
    std::cout << "Pruning luma threshold:   " << (m_lumaStdDev.value() * m_maxLumaError) << "\n";

    while (nonPrunedArea > (m_params.sampleBudget * percentageRatio / 100) &&
           m_lumaStdDev.value() < 1.0F) {
      std::cout << "Non-pruned exceeds " << percentageRatio << "% of total sample budget ("
                << (100.0 * nonPrunedArea / static_cast<double>(m_params.sampleBudget)) << "%)\n";
      std::cout << "Pruning luma threshold changed\n";

      *m_lumaStdDev *= 1.5F;
      if (m_lumaStdDev.value() > 1.0F) {
        m_lumaStdDev.emplace(1.0F);
      }

      m_reviveRatio = static_cast<int32_t>(100.F * ((1.F - m_lumaStdDev.value()) * A + 0.5F));
      prepareFrame(views);
      nonPrunedArea = pruneFrame(views);

      std::cout << "Pruning luma threshold:   " << (m_lumaStdDev.value() * m_maxLumaError) << "\n";
      std::cout << "reviveRatio: " << m_reviveRatio << "\n";
    }
  }

  void prepareFrame(const Common::DeepFrameList &views) {
    createInitialMasks(views);
    createSynthesizerPerPartialView(views);
    synthesizeReferenceViews(views);
  }

  void createInitialMasks(const Common::DeepFrameList &views) {
    m_masks.clear();
    m_masks.reserve(views.size());
    std::transform(std::cbegin(m_params.viewParamsList), std::cend(m_params.viewParamsList),
                   std::cbegin(views), back_inserter(m_masks),
                   [](const MivBitstream::ViewParams &viewParams, const Common::DeepFrame &view) {
                     auto mask = Common::Frame<uint8_t>::lumaOnly(
                         {viewParams.ci.ci_projection_plane_width_minus1() + 1,
                          viewParams.ci.ci_projection_plane_height_minus1() + 1});

                     std::transform(std::cbegin(view.geometry.getPlane(0)),
                                    std::cend(view.geometry.getPlane(0)),
                                    std::begin(mask.getPlane(0)),
                                    [ot = MivBitstream::OccupancyTransform{viewParams}](auto x) {
                                      // #94: When there are invalid pixels in a basic view, these
                                      // should be excluded from the pruning mask
                                      return ot.occupant(x) ? uint8_t{255} : uint8_t{};
                                    });
                     return mask;
                   });

    m_status = m_masks;
  }

  void createSynthesizerPerPartialView(const Common::DeepFrameList &views) {
    m_synthesizers.clear();
    for (size_t i = 0; i < m_params.viewParamsList.size(); ++i) {
      if (!m_params.viewParamsList[i].isBasicView) {
        const auto geoBitDepth = views[i].geometry.getBitDepth();
        const auto depthTransform =
            MivBitstream::DepthTransform{m_params.viewParamsList[i].dq, geoBitDepth};

        m_synthesizers.emplace_back(std::make_unique<IncrementalSynthesizer>(
            m_config, m_params.viewParamsList[i].ci.projectionPlaneSize(), i,
            depthTransform.expandDepth(views[i].geometry), expandLuma(views[i].texture),
            expandTexture(yuv444(views[i].texture))));
      }
    }
  }

  void synthesizeReferenceViews(const Common::DeepFrameList &views) {
    if (m_synthesizers.empty()) {
      // Skip generation the meshes
      std::cout << "Nothing to prune: only basic views\n";
      return;
    }

    for (auto &cluster : m_clusters) {
      for (size_t i : cluster.basicViewId) {
        synthesizeViews(i, views[i], cluster.additionalViewId);
      }
    }
  }

  auto pruneFrame(const Common::DeepFrameList &views) -> int32_t {
    for (auto &cluster : m_clusters) {
      for (auto i : cluster.pruningOrder) {
        auto it = find_if(std::begin(m_synthesizers), std::end(m_synthesizers),
                          [i](const auto &s) { return s->index == i; });
        m_synthesizers.erase(it);
        synthesizeViews(i, views[i], cluster.additionalViewId);
      }
    }

    auto sumValues = 0.;
    for (const auto &mask : m_masks) {
      sumValues =
          std::accumulate(std::begin(mask.getPlane(0)), std::end(mask.getPlane(0)), sumValues);
    }
    const auto lumaSamplesPerFrame = 2. * sumValues / 255e6;
    std::cout << "Non-pruned luma samples per frame is " << lumaSamplesPerFrame << "M\n";

    return static_cast<int32_t>((lumaSamplesPerFrame * 1e6) / 2);
  }

  // Synthesize the specified view to all remaining partial views.
  //
  // Special care is taken to make a pruned (masked) mesh once and re-use that
  // multiple times.
  void synthesizeViews(size_t index, const Common::DeepFrame &view,
                       const std::vector<size_t> &viewIds) {
    const auto &vp = m_params.viewParamsList[index];
    if (vp.viewInpaintFlag) {
      std::cout << "Skipping inpainted view " << vp.name << '\n';
      return;
    }
    auto [ivertices, triangles, attributes] =
        unprojectPrunedView(view, m_params.viewParamsList[index], m_masks[index].getPlane(0));

    if (m_params.viewParamsList[index].isBasicView) {
      std::cout << "Basic view ";
    } else {
      std::cout << "Prune view ";
    }

    const auto prec = std::cout.precision(2);
    const auto flags = std::cout.setf(std::ios::fixed, std::ios::floatfield);
    std::cout << std::setw(2) << index << " (" << std::setw(3)
              << m_params.viewParamsList[index].name << "): " << ivertices.size() << " vertices ("
              << 100. * static_cast<double>(ivertices.size()) /
                     (static_cast<double>(view.texture.getWidth()) * view.texture.getHeight())
              << "% of full view)\n";
    std::cout.precision(prec);
    std::cout.setf(flags);

    for (auto &s : m_synthesizers) {
      if (Common::contains(viewIds, s->index)) {
        auto overtices =
            project(ivertices, m_params.viewParamsList[index], m_params.viewParamsList[s->index]);
        weightedSphere(m_params.viewParamsList[s->index].ci, overtices, triangles);
        s->rasterizer.submit(overtices, attributes, triangles);
        s->rasterizer.run();
        updateMask(*s);
      }
    }
  }

  // Visit all pixels
  template <typename F> static void forPixels(std::array<size_t, 2> sizes, F f) {
    for (int32_t i = 0; i < static_cast<int32_t>(sizes[0]); ++i) {
      for (int32_t j = 0; j < static_cast<int32_t>(sizes[1]); ++j) {
        f(i, j);
      }
    }
  }

  // Visit all pixel neighbors (in between 3 and 8)
  template <typename F>
  static auto forNeighbors(int32_t i, int32_t j, std::array<size_t, 2> sizes, F f) -> bool {
    const int32_t n1 = std::max(0, i - 1);
    const int32_t n2 = std::min(static_cast<int32_t>(sizes[0]), i + 2);
    const int32_t m1 = std::max(0, j - 1);
    const int32_t m2 = std::min(static_cast<int32_t>(sizes[1]), j + 2);

    for (int32_t n = n1; n < n2; ++n) {
      for (int32_t m = m1; m < m2; ++m) {
        if (!f(n, m)) {
          return false;
        }
      }
    }
    return true;
  }

  static auto erode(const Common::Mat<uint8_t> &mask) -> Common::Mat<uint8_t> {
    Common::Mat<uint8_t> result{mask.sizes()};
    forPixels(mask.sizes(), [&](int32_t i, int32_t j) {
      result(i, j) =
          forNeighbors(i, j, mask.sizes(), [&mask](int32_t n, int32_t m) { return mask(n, m) > 0; })
              ? 255
              : 0;
    });
    return result;
  }

  static auto dilate(const Common::Mat<uint8_t> &mask) -> Common::Mat<uint8_t> {
    Common::Mat<uint8_t> result{mask.sizes()};
    forPixels(mask.sizes(), [&](int32_t i, int32_t j) {
      result(i, j) = forNeighbors(i, j, mask.sizes(),
                                  [&mask](int32_t n, int32_t m) { return mask(n, m) == 0; })
                         ? 0
                         : 255;
    });
    return result;
  }

  static void calcSamples(const int32_t sampleSize, const std::vector<size_t> &nonPrunedPixIndices,
                          const TMIV::Common::Mat<TMIV::Common::Vec3f> &referenceRGB,
                          const TMIV::Common::Mat<TMIV::Common::Vec3f> &synthesizedRGB,
                          std::vector<size_t> &sampledIndices, std::vector<float> &weights) {
    struct VecRgb {
      VecRgb(float _r, float _g, float _b) : r{_r}, g{_g}, b{_b} {}
      float r{}, g{}, b{};
    };
    std::vector<std::vector<size_t>> sampledPixIndices(sampleSize * sampleSize * sampleSize);
    std::vector<VecRgb> meanColors(sampleSize * sampleSize * sampleSize, {0.F, 0.F, 0.F});
    for (size_t idx : nonPrunedPixIndices) {
      VecRgb ref{std::min(255.F, std::max(0.F, referenceRGB[idx][0]) * 255.F),
                 std::min(255.F, std::max(0.F, referenceRGB[idx][1]) * 255.F),
                 std::min(255.F, std::max(0.F, referenceRGB[idx][2]) * 255.F)};
      VecRgb synth{synthesizedRGB[idx][0] * 255.F, synthesizedRGB[idx][1] * 255.F,
                   synthesizedRGB[idx][2] * 255.F};
      int32_t idxBin =
          std::min(sampleSize - 1, static_cast<int32_t>(ref.r) * sampleSize / 256) * sampleSize *
              sampleSize +
          std::min(sampleSize - 1, static_cast<int32_t>(ref.g) * sampleSize / 256) * sampleSize +
          std::min(sampleSize - 1, static_cast<int32_t>(ref.b) * sampleSize / 256);
      sampledPixIndices[idxBin].push_back(idx);
      meanColors[idxBin].r += static_cast<float>(static_cast<int32_t>(synth.r));
      meanColors[idxBin].g += static_cast<float>(static_cast<int32_t>(synth.g));
      meanColors[idxBin].b += static_cast<float>(static_cast<int32_t>(synth.b));
    }
    for (int32_t i = 0; i < sampleSize * sampleSize * sampleSize; i++) {
      float minv = 3.F * 255.F * 255.F;
      size_t minIdx = sampledPixIndices[i].size();
      meanColors[i].r /= static_cast<float>(sampledPixIndices[i].size());
      meanColors[i].g /= static_cast<float>(sampledPixIndices[i].size());
      meanColors[i].b /= static_cast<float>(sampledPixIndices[i].size());

      for (size_t j = 0; j < sampledPixIndices[i].size(); j++) {
        size_t idx = sampledPixIndices[i][j];
        VecRgb d{meanColors[i].r - synthesizedRGB[idx][0] * 255.F,
                 meanColors[i].g - synthesizedRGB[idx][1] * 255.F,
                 meanColors[i].b - synthesizedRGB[idx][2] * 255.F};
        auto err = d.r * d.r + d.g * d.g + d.b * d.b;
        if (minv > err) {
          minv = err;
          minIdx = j;
        }
      }

      if (minIdx < sampledPixIndices[i].size()) {
        sampledIndices.push_back(sampledPixIndices[i][minIdx]);
        weights.push_back(static_cast<float>(sampledPixIndices[i].size()) /
                          static_cast<float>(nonPrunedPixIndices.size()));
      }
    }
  }

  [[nodiscard]] auto
  getColorInconsistencyMaskSampled(const Common::Mat<Common::Vec3f> &referenceYUV,
                                   const Common::Mat<Common::Vec3f> &synthesizedYUV,
                                   const Common::Mat<uint8_t> &prunedMask) const
      -> Common::Mat<uint8_t> {
    struct VecRgb {
      VecRgb(float _r, float _g, float _b) : r{_r}, g{_g}, b{_b} {}
      float r{}, g{}, b{};
    };

    const double eps = 1E-10;
    Common::Mat<uint8_t> result(prunedMask.sizes(), 0);
    const auto referenceRGB{createRgbImageFromYuvImage(referenceYUV)};
    const auto synthesizedRGB{createRgbImageFromYuvImage(synthesizedYUV)};
    const auto nonPrunedPixIndices{computeNonPrunedPixelIndices(synthesizedYUV, prunedMask)};
    if (nonPrunedPixIndices.empty()) {
      return result;
    }

    std::vector<size_t> sampledIndices;
    std::vector<float> weights;

    calcSamples(m_sampleSize, nonPrunedPixIndices, referenceRGB, synthesizedRGB, sampledIndices,
                weights);
    const auto x1{iterativeReweightedLeastSquaresOnNonPrunedPixels(
        sampledIndices, referenceRGB, synthesizedRGB, weights, 0, eps)};
    const auto x2{iterativeReweightedLeastSquaresOnNonPrunedPixels(
        sampledIndices, referenceRGB, synthesizedRGB, weights, 1, eps)};
    const auto x3{iterativeReweightedLeastSquaresOnNonPrunedPixels(
        sampledIndices, referenceRGB, synthesizedRGB, weights, 2, eps)};

    std::vector<int32_t> nonPrunedPixErrBinIdx(nonPrunedPixIndices.size());
    const int32_t numBinErrHistogram = 100;
    double errHistThreshExeedSampleNum = 0.F;
    std::vector<double> errHistogram(numBinErrHistogram, 0.F);
    for (size_t i = 0; i < nonPrunedPixIndices.size(); ++i) {
      size_t pixIdx = nonPrunedPixIndices[i];
      const VecRgb s{synthesizedRGB[pixIdx][0], synthesizedRGB[pixIdx][1],
                     synthesizedRGB[pixIdx][2]};
      const VecRgb c{static_cast<float>(computeWeightedRgbSum(referenceRGB[pixIdx], x1)),
                     static_cast<float>(computeWeightedRgbSum(referenceRGB[pixIdx], x2)),
                     static_cast<float>(computeWeightedRgbSum(referenceRGB[pixIdx], x3))};
      VecRgb d{std::abs(c.r - s.r), std::abs(c.g - s.g), std::abs(c.b - s.b)};
      auto err = std::max(d.r, std::max(d.g, d.b));

      int32_t errHistBinIdx =
          std::min(numBinErrHistogram - 1,
                   static_cast<int32_t>(err * static_cast<float>(numBinErrHistogram)));
      nonPrunedPixErrBinIdx[i] = errHistBinIdx;
      errHistogram[errHistBinIdx] += 1.F;

      if (err > m_maxColorError) {
        errHistThreshExeedSampleNum += 1.F;
        if (m_reviveRatio == 100) {
          result[pixIdx] = 255;
        }
      }
    }

    if (m_reviveRatio == 100) {
      return result;
    }

    int32_t cutBinIdx = numBinErrHistogram;
    float accumHist = 0.F;
    float minimumErr = 1.F / static_cast<float>(numBinErrHistogram);
    for (int32_t i = numBinErrHistogram - 1; i >= 0; i--) {
      accumHist += static_cast<float>(errHistogram[i] / errHistThreshExeedSampleNum);
      double curErr = minimumErr * static_cast<float>(i);
      if (accumHist < static_cast<float>(m_reviveRatio) / 100.F && curErr >= m_maxColorError) {
        cutBinIdx = i;
      } else {
        break;
      }
    }

    for (size_t i = 0; i < nonPrunedPixIndices.size(); i++) {
      size_t pixIdx = nonPrunedPixIndices[i];
      if (nonPrunedPixErrBinIdx[i] >= cutBinIdx) {
        result[pixIdx] = 255;
      }
    }

    return result;
  }

  [[nodiscard]] auto getColorInconsistencyMask(const Common::Mat<Common::Vec3f> &referenceYUV,
                                               const Common::Mat<Common::Vec3f> &synthesizedYUV,
                                               const Common::Mat<uint8_t> &prunedMask) const
      -> Common::Mat<uint8_t> {
    struct VecRgb {
      VecRgb(double _r, double _g, double _b) : r{_r}, g{_g}, b{_b} {}
      double r{}, g{}, b{};
    };

    const int32_t maxIterNum = 10;
    const double eps = 1E-10;
    Common::Mat<uint8_t> result(prunedMask.sizes(), 0);
    const auto referenceRGB{createRgbImageFromYuvImage(referenceYUV)};
    const auto synthesizedRGB{createRgbImageFromYuvImage(synthesizedYUV)};

    const auto nonPrunedPixIndices{computeNonPrunedPixelIndices(synthesizedYUV, prunedMask)};

    if (nonPrunedPixIndices.empty()) {
      return result;
    }

    size_t numPixels = nonPrunedPixIndices.size();
    std::vector<float> weightR(numPixels, 1.F);
    std::vector<float> weightG(numPixels, 1.F);
    std::vector<float> weightB(numPixels, 1.F);

    double prevE = -1.0;
    for (int32_t iter = 0; iter < maxIterNum; ++iter) {
      const auto x1{iterativeReweightedLeastSquaresOnNonPrunedPixels(
          nonPrunedPixIndices, referenceRGB, synthesizedRGB, weightR, 0, eps)};
      const auto x2{iterativeReweightedLeastSquaresOnNonPrunedPixels(
          nonPrunedPixIndices, referenceRGB, synthesizedRGB, weightG, 1, eps)};
      const auto x3{iterativeReweightedLeastSquaresOnNonPrunedPixels(
          nonPrunedPixIndices, referenceRGB, synthesizedRGB, weightB, 2, eps)};

      double curE = 0;
      double weightSum = 0;
      for (size_t i = 0; i < numPixels; ++i) {
        size_t pixIdx = nonPrunedPixIndices[i];
        const VecRgb s{static_cast<double>(synthesizedRGB[pixIdx][0]),
                       static_cast<double>(synthesizedRGB[pixIdx][1]),
                       static_cast<double>(synthesizedRGB[pixIdx][2])};

        const VecRgb c{computeWeightedRgbSum(referenceRGB[pixIdx], x1),
                       computeWeightedRgbSum(referenceRGB[pixIdx], x2),
                       computeWeightedRgbSum(referenceRGB[pixIdx], x3)};

        VecRgb d{std::abs(c.r - s.r), std::abs(c.g - s.g), std::abs(c.b - s.b)};

        result[pixIdx] = 0;
        if (d.r > m_maxColorError || d.g > m_maxColorError || d.b > m_maxColorError) {
          result[pixIdx] = 255;
        }

        const VecRgb w{static_cast<double>(weightR[i]), static_cast<double>(weightG[i]),
                       static_cast<double>(weightB[i])};

        curE += (w.r * d.r + w.g * d.g + w.b * d.b) / 3.0;
        weightSum += (w.r + w.g + w.b) / 3.0;

        using std::sqrt;
        d.r = sqrt(d.r * d.r + eps);
        d.g = sqrt(d.g * d.g + eps);
        d.b = sqrt(d.b * d.b + eps);

        weightR[i] = static_cast<float>(std::min(1.0 / (2.0 * d.r), 1.0));
        weightG[i] = static_cast<float>(std::min(1.0 / (2.0 * d.g), 1.0));
        weightB[i] = static_cast<float>(std::min(1.0 / (2.0 * d.b), 1.0));
      }

      curE /= weightSum;
      if (prevE < 0) {
        prevE = curE;
        continue;
      }
      double update = std::abs(prevE - curE);
      if (update < eps) {
        break;
      }
      prevE = curE;
    }
    return result;
  }

  void updateMask(IncrementalSynthesizer &synthesizer) {
    auto &mask = m_masks[synthesizer.index].getPlane(0);
    auto &status = m_status[synthesizer.index].getPlane(0);
    Common::Mat<uint8_t> colorInconsistencyMask;
    Common::Mat<uint8_t>::iterator iColor;
    if (m_enable2ndPassPruner) {
      if (m_sampleSize == 0) {
        colorInconsistencyMask = getColorInconsistencyMask(
            synthesizer.referenceYUV, synthesizer.rasterizer.attribute<0>(), mask);
      } else {
        colorInconsistencyMask = getColorInconsistencyMaskSampled(
            synthesizer.referenceYUV, synthesizer.rasterizer.attribute<0>(), mask);
      }
      iColor = std::begin(colorInconsistencyMask);
    }

    auto i = std::begin(mask);
    auto j = std::begin(synthesizer.reference);
    auto jY = std::begin(synthesizer.referenceY);
    auto k = std::begin(status);

    int32_t pp = 0;
    const auto W = static_cast<int32_t>(synthesizer.reference.width());
    const auto H = static_cast<int32_t>(synthesizer.reference.height());

    auto modifiedMaxLumaError = m_maxLumaError * m_lumaStdDev.value();

    synthesizer.rasterizer.visit([&](const Renderer::PixelValue<Common::Vec3f> &x) {
      if (x.normDisp > 0) {
        const auto depthError = (x.depth() / *j - 1.F);
        auto lumaError = std::abs(std::get<0>(x.attributes()).x() - *(jY));

        const auto h = pp / W;
        const auto w = pp % W;

        for (int32_t hh = -1; hh <= 1; hh++) {
          for (int32_t ww = -1; ww <= 1; ww++) {
            if (h + hh < 0 || h + hh >= H || w + ww < 0 || w + ww >= W) {
              continue;
            }
            const auto offset = hh * W + ww;
            lumaError =
                std::min(lumaError, std::abs(std::get<0>(x.attributes()).x() - *(jY + offset)));
          }
        }

        if (std::abs(depthError) < m_maxDepthError && lumaError < modifiedMaxLumaError) {
          if (*k != 0) {
            *i = 0;
          }
        } else if (m_params.depthLowQualityFlag && depthError < -depthErrorEps) {
          if (*k != 0) {
            *k = 0;
            *i = 255;
          }
        }

        if (m_enable2ndPassPruner && *iColor > 0) {
          *i = 255;
        }
      }

      i++;
      j++;
      jY++;
      k++;
      pp++;
      if (m_enable2ndPassPruner) {
        iColor++;
      }

      return true;
    });
    for (int32_t n = 0; n < m_erode; ++n) {
      mask = erode(mask);
    }
    for (int32_t n = 0; n < m_dilate; ++n) {
      mask = dilate(mask);
    }
    synthesizer.maskAverage =
        static_cast<float>(std::accumulate(std::begin(mask), std::end(mask), 0)) /
        (2.55F * static_cast<float>(mask.width() * mask.height()));
  }
};

HierarchicalPruner::HierarchicalPruner(const Common::Json & /* unused */,
                                       const Common::Json &nodeConfig)
    : m_impl(new Impl{nodeConfig}) {}

HierarchicalPruner::~HierarchicalPruner() = default;

auto HierarchicalPruner::prepareSequence(const PrunerParams &params)
    -> std::vector<MivBitstream::PruningParents> {
  return m_impl->prepareSequence(params);
}

auto HierarchicalPruner::prune(const MivBitstream::ViewParamsList &viewParamsList,
                               const Common::DeepFrameList &views) -> Common::FrameList<uint8_t> {
  return m_impl->prune(viewParamsList, views);
}
} // namespace TMIV::Pruner
