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

#include <TMIV/Pruner/HierarchicalPruner.h>

#include "PrunedMesh.h"
#include <TMIV/Common/Graph.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <TMIV/Renderer/Rasterizer.h>
#include <TMIV/Renderer/reprojectPoints.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <future>
#include <iomanip>
#include <iostream>
#include <numeric>

namespace TMIV::Pruner {
class HierarchicalPruner::Impl {
private:
  struct IncrementalSynthesizer {
    IncrementalSynthesizer(const Renderer::AccumulatingPixel<Common::Vec3f> &config,
                           Common::Vec2i size, size_t index_, Common::Mat<float> reference_,
                           Common::Mat<float> referenceY_)
        : rasterizer{config, size}
        , index{index_}
        , reference{std::move(reference_)}
        , referenceY{std::move(referenceY_)} {}

    Renderer::Rasterizer<Common::Vec3f> rasterizer;
    const size_t index;
    float maskAverage{0.F};
    const Common::Mat<float> reference;
    const Common::Mat<float> referenceY;
  };

  const float m_maxDepthError{};
  const float m_maxLumaError{};
  const float m_maxStretching{};
  const int m_erode{};
  const int m_dilate{};
  const int m_maxBasicViewsPerGraph{};
  const Renderer::AccumulatingPixel<Common::Vec3f> m_config;
  MivBitstream::EncoderParams m_params;
  std::vector<std::unique_ptr<IncrementalSynthesizer>> m_synthesizers;
  std::vector<size_t> m_clusterIds;
  struct Cluster {
    std::vector<size_t> basicViewId;
    std::vector<size_t> additionalViewId;
    std::vector<size_t> pruningOrder;
  };
  std::vector<Cluster> m_clusters;
  std::vector<Common::Frame<Common::YUV400P8>> m_masks;
  std::vector<Common::Frame<Common::YUV400P8>> m_status;

public:
  explicit Impl(const Common::Json &nodeConfig)
      : m_maxDepthError{nodeConfig.require("maxDepthError").asFloat()}
      , m_maxLumaError{nodeConfig.require("maxLumaError").asFloat()}
      , m_maxStretching{nodeConfig.require("maxStretching").asFloat()}
      , m_erode{nodeConfig.require("erode").asInt()}
      , m_dilate{nodeConfig.require("dilate").asInt()}
      , m_maxBasicViewsPerGraph{nodeConfig.require("maxBasicViewsPerGraph").asInt()}
      , m_config{nodeConfig.require("rayAngleParameter").asFloat(),
                 nodeConfig.require("depthParameter").asFloat(),
                 nodeConfig.require("stretchingParameter").asFloat(), m_maxStretching} {}

  void assignAdditionalViews(const Common::Mat<float> &overlap,
                             const MivBitstream::ViewParamsList &viewParamsList, size_t numClusters,
                             std::vector<size_t> &clusterIds) {
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

  auto scoreClustering(const Common::Mat<float> &overlap, const std::vector<size_t> &clusterIds)
      -> double {
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

  auto exhaustiveSearch(const Common::Mat<float> &overlap,
                        const MivBitstream::ViewParamsList &viewParamsList) -> std::vector<size_t> {
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
    assert(numClusters >= 1);

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

    assert(!bestClusterIds.empty());
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
    for (auto &cluster : m_clusters) {
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

  void registerPruningRelation(MivBitstream::EncoderParams &params) {
    auto &viewParamsList = params.viewParamsList;
    Renderer::ProjectionHelperList cameraHelperList{viewParamsList};

    // Create clusters and pruning order
    auto overlappingMatrix = computeOverlappingMatrix(cameraHelperList);
    clusterViews(overlappingMatrix, viewParamsList);
    computePruningOrder(overlappingMatrix);
    printClusters(viewParamsList);

    // Pruning graph
    Common::Graph::BuiltIn::Sparse<float> pruningGraph(viewParamsList.size());

    for (auto &cluster : m_clusters) {
      if (!cluster.pruningOrder.empty()) {
        for (auto i : cluster.basicViewId) {
          pruningGraph.connect(cluster.pruningOrder.front(), i, 1.F,
                               Common::Graph::LinkType::Directed);
        }
        for (size_t i = 1; i < cluster.pruningOrder.size(); ++i) {
          pruningGraph.connect(cluster.pruningOrder[i], cluster.pruningOrder[i - 1], 1.F,
                               Common::Graph::LinkType::Directed);
        }
      }
    }

    // Pruning mask
    for (auto camId = 0U; camId < viewParamsList.size(); camId++) {
      const auto &neighbourhood = pruningGraph.getNeighbourhood(camId);

      if (neighbourhood.empty()) {
        viewParamsList[camId].pp = MivBitstream::PruningParents{};
      } else {
        std::vector<uint16_t> parentIdList;

        parentIdList.reserve(neighbourhood.size());

        for (const auto &link : neighbourhood) {
          parentIdList.emplace_back(static_cast<uint16_t>(link.node()));
        }

        viewParamsList[camId].pp = MivBitstream::PruningParents{std::move(parentIdList)};
      }
    }
  }

  auto prune(const MivBitstream::EncoderParams &params, const Common::MVD16Frame &views,
             const int blockSize) -> Common::MaskList {
    m_params = params;

    prepareFrame(views, blockSize);
    pruneFrame(views);

    return std::move(m_masks);
  }

private:
  void prepareFrame(const Common::MVD16Frame &views, const int blockSize) {
    createInitialMasks(views, blockSize);
    createSynthesizerPerPartialView(views);
    synthesizeReferenceViews(views);
  }

  void createInitialMasks(const Common::MVD16Frame &views, const int blockSize) {
    m_masks.clear();
    m_masks.reserve(views.size());
    std::transform(std::cbegin(m_params.viewParamsList), std::cend(m_params.viewParamsList),
                   std::cbegin(views), back_inserter(m_masks),
                   [blockSize](const MivBitstream::ViewParams &viewParams,
                               const Common::TextureDepth16Frame &view) {
                     auto mask = Common::Frame<Common::YUV400P8>{
                         Common::align(viewParams.ci.projectionPlaneSize().x(), blockSize),
                         Common::align(viewParams.ci.projectionPlaneSize().y(), blockSize)};

                     std::transform(std::cbegin(view.depth.getPlane(0)),
                                    std::cend(view.depth.getPlane(0)), std::begin(mask.getPlane(0)),
                                    [ot = MivBitstream::OccupancyTransform{viewParams}](auto x) {
                                      // #94: When there are invalid pixels in a basic view, these
                                      // should be excluded from the pruning mask
                                      return uint8_t(ot.occupant(x) ? 255 : 0);
                                    });
                     return mask;
                   });

    m_status.clear();
    m_status.reserve(views.size());
    std::transform(std::cbegin(m_params.viewParamsList), std::cend(m_params.viewParamsList),
                   std::cbegin(views), back_inserter(m_status),
                   [blockSize](const MivBitstream::ViewParams &viewParams,
                               const Common::TextureDepth16Frame &view) {
                     auto status = Common::Frame<Common::YUV400P8>{
                         Common::align(viewParams.ci.projectionPlaneSize().x(), blockSize),
                         Common::align(viewParams.ci.projectionPlaneSize().y(), blockSize)};

                     std::transform(std::cbegin(view.depth.getPlane(0)),
                                    std::cend(view.depth.getPlane(0)),
                                    std::begin(status.getPlane(0)),
                                    [ot = MivBitstream::OccupancyTransform{viewParams}](auto x) {
                                      // #94: When there are invalid pixels in a basic view, these
                                      // should be freezed from pruning
                                      return uint8_t(ot.occupant(x) ? 255 : 0);
                                    });
                     return status;
                   });
  }

  void createSynthesizerPerPartialView(const Common::MVD16Frame &views) {
    m_synthesizers.clear();
    for (size_t i = 0; i < m_params.viewParamsList.size(); ++i) {
      if (!m_params.viewParamsList[i].isBasicView) {
        const auto depthTransform = MivBitstream::DepthTransform<16>{m_params.viewParamsList[i].dq};
        m_synthesizers.emplace_back(std::make_unique<IncrementalSynthesizer>(
            m_config, m_params.viewParamsList[i].ci.projectionPlaneSize(), i,
            depthTransform.expandDepth(views[i].depth), expandLuma(views[i].texture)));
      }
    }
  }

  void synthesizeReferenceViews(const Common::MVD16Frame &views) {
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

  void pruneFrame(const Common::MVD16Frame &views) {
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
  }

  // Synthesize the specified view to all remaining partial views.
  //
  // Special care is taken to make a pruned (masked) mesh once and re-use that
  // multiple times.
  void synthesizeViews(size_t index, const Common::TextureDepth16Frame &view,
                       const std::vector<size_t> &viewIds) {
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
              << 100. * double(ivertices.size()) /
                     (double(view.texture.getWidth()) * view.texture.getHeight())
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
    for (int i = 0; i < int(sizes[0]); ++i) {
      for (int j = 0; j < int(sizes[1]); ++j) {
        f(i, j);
      }
    }
  }

  // Visit all pixel neighbors (in between 3 and 8)
  template <typename F>
  static auto forNeighbors(int i, int j, std::array<size_t, 2> sizes, F f) -> bool {
    const int n1 = std::max(0, i - 1);
    const int n2 = std::min(int(sizes[0]), i + 2);
    const int m1 = std::max(0, j - 1);
    const int m2 = std::min(int(sizes[1]), j + 2);

    for (int n = n1; n < n2; ++n) {
      for (int m = m1; m < m2; ++m) {
        if (!f(n, m)) {
          return false;
        }
      }
    }
    return true;
  }

  static auto erode(const Common::Mat<uint8_t> &mask) -> Common::Mat<uint8_t> {
    Common::Mat<uint8_t> result{mask.sizes()};
    forPixels(mask.sizes(), [&](int i, int j) {
      result(i, j) =
          forNeighbors(i, j, mask.sizes(), [&mask](int n, int m) { return mask(n, m) > 0; }) ? 255
                                                                                             : 0;
    });
    return result;
  }

  static auto dilate(const Common::Mat<uint8_t> &mask) -> Common::Mat<uint8_t> {
    Common::Mat<uint8_t> result{mask.sizes()};
    forPixels(mask.sizes(), [&](int i, int j) {
      result(i, j) =
          forNeighbors(i, j, mask.sizes(), [&mask](int n, int m) { return mask(n, m) == 0; }) ? 0
                                                                                              : 255;
    });
    return result;
  }

  void updateMask(IncrementalSynthesizer &synthesizer) {
    auto &mask = m_masks[synthesizer.index].getPlane(0);
    auto &status = m_status[synthesizer.index].getPlane(0);

    auto i = std::begin(mask);
    auto j = std::begin(synthesizer.reference);
    auto jY = std::begin(synthesizer.referenceY);
    auto k = std::begin(status);

    int pp = 0;
    const auto W = int(synthesizer.reference.width());
    const auto H = int(synthesizer.reference.height());

    synthesizer.rasterizer.visit([&](const Renderer::PixelValue<Common::Vec3f> &x) {
      if (x.normDisp > 0) {
        const auto depthError = (x.depth() / *j - 1.F);
        auto lumaError = std::abs(std::get<0>(x.attributes()).x() - *(jY));

        const auto h = pp / W;
        const auto w = pp % W;

        for (int hh = -1; hh <= 1; hh++) {
          for (int ww = -1; ww <= 1; ww++) {
            if (h + hh < 0 || h + hh >= H || w + ww < 0 || w + ww >= W) {
              continue;
            }
            const auto offset = hh * W + ww;
            lumaError =
                std::min(lumaError, std::abs(std::get<0>(x.attributes()).x() - *(jY + offset)));
          }
        }

        if (std::abs(depthError) < m_maxDepthError && lumaError < m_maxLumaError) {
          if (*k != 0) {
            *i = 0;
          }
        } else if (m_params.vme().vme_depth_low_quality_flag() && (depthError < 0.F)) {
          if (*k != 0) {
            *k = 0;
            *i = 255;
          }
        }
      }

      i++;
      j++;
      jY++;
      k++;
      pp++;

      return true;
    });
    for (int n = 0; n < m_erode; ++n) {
      mask = erode(mask);
    }
    for (int n = 0; n < m_dilate; ++n) {
      mask = dilate(mask);
    }
    synthesizer.maskAverage = float(std::accumulate(std::begin(mask), std::end(mask), 0)) /
                              (2.55F * float(mask.width() * mask.height()));
  }
};

HierarchicalPruner::HierarchicalPruner(const Common::Json & /* unused */,
                                       const Common::Json &nodeConfig)
    : m_impl(new Impl{nodeConfig}) {}

HierarchicalPruner::~HierarchicalPruner() = default;

void HierarchicalPruner::registerPruningRelation(MivBitstream::EncoderParams &params) {
  return m_impl->registerPruningRelation(params);
}

auto HierarchicalPruner::prune(const MivBitstream::EncoderParams &params,
                               const Common::MVD16Frame &views, const int blockSize)
    -> Common::MaskList {
  return m_impl->prune(params, views, blockSize);
}
} // namespace TMIV::Pruner
