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

using namespace TMIV::Common;
using namespace TMIV::Common::Graph;
using namespace TMIV::MivBitstream;
using namespace TMIV::Renderer;
using namespace std;

namespace TMIV::Pruner {
class HierarchicalPruner::Impl {
private:
  struct IncrementalSynthesizer {
    IncrementalSynthesizer(const AccumulatingPixel<Vec3f> &config, Vec2i size, size_t index_,
                           Mat<float> reference_)
        : rasterizer{config, size}, index{index_}, reference{move(reference_)} {}

    Rasterizer<Vec3f> rasterizer;
    const size_t index;
    float maskAverage{0.F};
    const Mat<float> reference;
  };

  const float m_maxDepthError{};
  const float m_maxStretching{};
  const int m_erode{};
  const int m_dilate{};
  const int m_maxBasicViewsPerGraph{};
  const AccumulatingPixel<Vec3f> m_config;
  IvSequenceParams m_ivSequenceParams;
  vector<bool> m_isBasicView;
  vector<unique_ptr<IncrementalSynthesizer>> m_synthesizers;
  vector<size_t> m_clusterIds;
  struct Cluster {
    vector<size_t> basicViewId;
    vector<size_t> additionalViewId;
    vector<size_t> pruningOrder;
  };
  vector<Cluster> m_clusters;
  vector<Frame<YUV400P8>> m_masks;
  vector<Frame<YUV400P8>> m_status;

public:
  explicit Impl(const Json &nodeConfig)
      : m_maxDepthError{nodeConfig.require("maxDepthError").asFloat()}
      , m_maxStretching{nodeConfig.require("maxStretching").asFloat()}
      , m_erode{nodeConfig.require("erode").asInt()}
      , m_dilate{nodeConfig.require("dilate").asInt()}
      , m_maxBasicViewsPerGraph{nodeConfig.require("maxBasicViewsPerGraph").asInt()}
      , m_config{nodeConfig.require("rayAngleParameter").asFloat(),
                 nodeConfig.require("depthParameter").asFloat(),
                 nodeConfig.require("stretchingParameter").asFloat(), m_maxStretching} {}

  void assignAdditionalViews(const Mat<float> &overlap, const vector<bool> &isBasicView,
                             size_t numClusters, vector<size_t> &clusterIds) {
    const auto N = isBasicView.size();
    auto numViewsPerCluster = vector<size_t>(numClusters, 0);
    for (size_t i = 0; i < N; ++i) {
      if (isBasicView[i]) {
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
        if (isBasicView[i]) {
          for (size_t j = 0; j < N; ++j) {
            if (!isBasicView[j] && clusterIds[j] == numClusters) {
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

  auto scoreClustering(const Mat<float> &overlap, const vector<bool> &isBasicView,
                       const vector<size_t> &clusterIds) -> double {
    auto score = 0.;
    const auto N = isBasicView.size();

    for (size_t i = 0; i < N; ++i) {
      for (size_t j = i + 1; j < N; ++j) {
        if (clusterIds[i] == clusterIds[j]) {
          score += overlap(i, j);
        }
      }
    }
    return score;
  }

  auto exhaustiveSearch(const Mat<float> &overlap, const vector<bool> &isBasicView)
      -> vector<size_t> {
    auto basicViewIds = vector<size_t>{};
    for (size_t i = 0; i < isBasicView.size(); ++i) {
      if (isBasicView[i]) {
        basicViewIds.push_back(i);
      }
    }

    const size_t maxBasicViews = m_maxBasicViewsPerGraph;
    const auto numClusters = (basicViewIds.size() + maxBasicViews - 1) / maxBasicViews;
    assert(numClusters >= 1);

    size_t numPermutations = 1;
    for (size_t i = 0; i < basicViewIds.size(); ++i) {
      numPermutations *= numClusters;
    }

    auto clusterIds = vector<size_t>(isBasicView.size());
    auto numBasicViewsPerCluster = vector<size_t>(numClusters);

    auto bestScore = 0.;
    auto bestClusterIds = vector<size_t>{};

    for (size_t p = 0; p < numPermutations; ++p) {
      auto q = p;
      fill(clusterIds.begin(), clusterIds.end(), numClusters);
      fill(numBasicViewsPerCluster.begin(), numBasicViewsPerCluster.end(), 0);
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
        assignAdditionalViews(overlap, isBasicView, numClusters, clusterIds);
        const auto score = scoreClustering(overlap, isBasicView, clusterIds);

        if (bestScore < score) {
          bestScore = score;
          bestClusterIds = clusterIds;
        }
      }
    }

    assert(!bestClusterIds.empty());
    return bestClusterIds;
  }

  void clusterViews(const Mat<float> &overlap, const vector<bool> &isBasicView) {
    const auto clusterIds = exhaustiveSearch(overlap, isBasicView);

    m_clusters = vector<Cluster>(1 + *max_element(clusterIds.cbegin(), clusterIds.cend()));
    for (size_t i = 0; i < clusterIds.size(); ++i) {
      if (isBasicView[i]) {
        m_clusters[clusterIds[i]].basicViewId.push_back(i);
      } else {
        m_clusters[clusterIds[i]].additionalViewId.push_back(i);
      }
    }
  }

  void computePruningOrder(const Mat<float> &overlappingMatrix) {
    for (auto &cluster : m_clusters) {
      auto processedList = cluster.basicViewId;
      auto pendingList = cluster.additionalViewId;
      cluster.pruningOrder.clear();

      while (!pendingList.empty()) {
        float worseOverlapping = numeric_limits<float>::max();
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

  void printClusters(const ViewParamsList &vpl) const {
    cout << "Pruning graph:\n";
    for (auto &cluster : m_clusters) {
      cout << "  (";
      for (auto i : cluster.basicViewId) {
        cout << ' ' << vpl[i].name;
      }
      cout << " )";
      for (auto i : cluster.pruningOrder) {
        cout << " <- " << vpl[i].name;
      }
      cout << '\n';
    }
  }

  void registerPruningRelation(MivBitstream::IvSequenceParams &ivSequenceParams,
                               const vector<bool> &isBasicView) {
    auto &viewParamsList = ivSequenceParams.viewParamsList;
    ProjectionHelperList cameraHelperList{viewParamsList};

    // Create clusters and pruning order
    auto overlappingMatrix = computeOverlappingMatrix(cameraHelperList);
    clusterViews(overlappingMatrix, isBasicView);
    computePruningOrder(overlappingMatrix);
    printClusters(viewParamsList);

    // Pruning graph
    Graph::BuiltIn::Sparse<float> pruningGraph(viewParamsList.size());

    for (auto &cluster : m_clusters) {
      if (!cluster.pruningOrder.empty()) {
        for (auto i : cluster.basicViewId) {
          pruningGraph.connect(cluster.pruningOrder.front(), i, 1.F, LinkType::Directed);
        }
        for (size_t i = 1; i < cluster.pruningOrder.size(); ++i) {
          pruningGraph.connect(cluster.pruningOrder[i], cluster.pruningOrder[i - 1], 1.F,
                               LinkType::Directed);
        }
      }
    }

    // Pruning mask
    for (auto camId = 0U; camId < viewParamsList.size(); camId++) {
      const auto &neighbourhood = pruningGraph.getNeighbourhood(camId);

      if (neighbourhood.empty()) {
        viewParamsList[camId].pp = PruningParent{};
      } else {
        vector<uint16_t> parentIdList;

        parentIdList.reserve(neighbourhood.size());

        for (const auto &link : neighbourhood) {
          parentIdList.emplace_back(static_cast<uint16_t>(link.node()));
        }

        viewParamsList[camId].pp = PruningParent{move(parentIdList)};
      }
    }
  }

  auto prune(const MivBitstream::IvSequenceParams &ivSequenceParams, const MVD16Frame &views,
             const vector<bool> &isBasicView) -> MaskList {
    m_ivSequenceParams = ivSequenceParams;
    m_isBasicView = isBasicView;

    prepareFrame(views);
    pruneFrame(views);

    return move(m_masks);
  }

private:
  void prepareFrame(const MVD16Frame &views) {
    createInitialMasks(views);
    createSynthesizerPerPartialView(views);
    synthesizeReferenceViews(views);
  }

  void createInitialMasks(const MVD16Frame &views) {
    m_masks.clear();
    m_masks.reserve(views.size());
    transform(cbegin(m_ivSequenceParams.viewParamsList), cend(m_ivSequenceParams.viewParamsList),
              cbegin(views), back_inserter(m_masks),
              [](const ViewParams &viewParams, const TextureDepth16Frame &view) {
                auto mask = Frame<YUV400P8>{viewParams.ci.projectionPlaneSize().x(),
                                            viewParams.ci.projectionPlaneSize().y()};
                transform(cbegin(view.depth.getPlane(0)), cend(view.depth.getPlane(0)),
                          begin(mask.getPlane(0)), [ot = OccupancyTransform{viewParams}](auto x) {
                            // #94: When there are invalid pixels in a basic view, these should be
                            // excluded from the pruning mask
                            return uint8_t(ot.occupant(x) ? 255 : 0);
                          });
                return mask;
              });

    m_status.clear();
    m_status.reserve(views.size());
    transform(cbegin(m_ivSequenceParams.viewParamsList), cend(m_ivSequenceParams.viewParamsList),
              cbegin(views), back_inserter(m_status),
              [](const ViewParams &viewParams, const TextureDepth16Frame &view) {
                auto status = Frame<YUV400P8>{viewParams.ci.projectionPlaneSize().x(),
                                              viewParams.ci.projectionPlaneSize().y()};
                transform(cbegin(view.depth.getPlane(0)), cend(view.depth.getPlane(0)),
                          begin(status.getPlane(0)), [ot = OccupancyTransform{viewParams}](auto x) {
                            // #94: When there are invalid pixels in a basic view, these should be
                            // freezed from pruning
                            return uint8_t(ot.occupant(x) ? 255 : 0);
                          });
                return status;
              });
  }

  void createSynthesizerPerPartialView(const MVD16Frame &views) {
    m_synthesizers.clear();
    for (size_t i = 0; i < m_ivSequenceParams.viewParamsList.size(); ++i) {
      if (!m_isBasicView[i]) {
        const auto depthTransform = DepthTransform<16>{m_ivSequenceParams.viewParamsList[i].dq};
        m_synthesizers.emplace_back(make_unique<IncrementalSynthesizer>(
            m_config, m_ivSequenceParams.viewParamsList[i].ci.projectionPlaneSize(), i,
            depthTransform.expandDepth(views[i].depth)));
      }
    }
  }

  void synthesizeReferenceViews(const MVD16Frame &views) {
    if (m_synthesizers.empty()) {
      // Skip generation the meshes
      cout << "Nothing to prune: only basic views\n";
      return;
    }

    for (auto &cluster : m_clusters) {
      for (size_t i : cluster.basicViewId) {
        synthesizeViews(i, views[i], cluster.additionalViewId);
      }
    }
  }

  void pruneFrame(const MVD16Frame &views) {
    for (auto &cluster : m_clusters) {
      for (auto i : cluster.pruningOrder) {
        auto it = find_if(begin(m_synthesizers), end(m_synthesizers),
                          [i](const auto &s) { return s->index == i; });
        m_synthesizers.erase(it);
        synthesizeViews(i, views[i], cluster.additionalViewId);
      }
    }

    auto sumValues = 0.;
    for (const auto &mask : m_masks) {
      sumValues = accumulate(begin(mask.getPlane(0)), end(mask.getPlane(0)), sumValues);
    }
    const auto lumaSamplesPerFrame = 2. * sumValues / 255e6;
    cout << "Non-pruned luma samples per frame is " << lumaSamplesPerFrame << "M\n";
  }

  // Synthesize the specified view to all remaining partial views.
  //
  // Special care is taken to make a pruned (masked) mesh once and re-use that
  // multiple times.
  void synthesizeViews(size_t index, const TextureDepth16Frame &view,
                       const vector<size_t> &viewIds) {
    auto [ivertices, triangles, attributes] = unprojectPrunedView(
        view, m_ivSequenceParams.viewParamsList[index], m_masks[index].getPlane(0));

    if (m_isBasicView[index]) {
      cout << "Basic view ";
    } else {
      cout << "Prune view ";
    }

    const auto prec = cout.precision(2);
    const auto flags = cout.setf(ios::fixed, ios::floatfield);
    cout << setw(2) << index << " (" << setw(3) << m_ivSequenceParams.viewParamsList[index].name
         << "): " << ivertices.size() << " vertices ("
         << 100. * double(ivertices.size()) / (view.texture.getWidth() * view.texture.getHeight())
         << "% of full view)\n";
    cout.precision(prec);
    cout.setf(flags);

    for (auto &s : m_synthesizers) {
      if (contains(viewIds, s->index)) {
        auto overtices = project(ivertices, m_ivSequenceParams.viewParamsList[index],
                                 m_ivSequenceParams.viewParamsList[s->index]);
        weightedSphere(m_ivSequenceParams.viewParamsList[s->index].ci, overtices, triangles);
        s->rasterizer.submit(overtices, attributes, triangles);
        s->rasterizer.run();
        updateMask(*s);
      }
    }
  }

  // Visit all pixels
  template <typename F> static void forPixels(array<size_t, 2> sizes, F f) {
    for (int i = 0; i < int(sizes[0]); ++i) {
      for (int j = 0; j < int(sizes[1]); ++j) {
        f(i, j);
      }
    }
  }

  // Visit all pixel neighbors (in between 3 and 8)
  template <typename F>
  static auto forNeighbors(int i, int j, array<size_t, 2> sizes, F f) -> bool {
    const int n1 = max(0, i - 1);
    const int n2 = min(int(sizes[0]), i + 2);
    const int m1 = max(0, j - 1);
    const int m2 = min(int(sizes[1]), j + 2);

    for (int n = n1; n < n2; ++n) {
      for (int m = m1; m < m2; ++m) {
        if (!f(n, m)) {
          return false;
        }
      }
    }
    return true;
  }

  static auto erode(const Mat<uint8_t> &mask) -> Mat<uint8_t> {
    Mat<uint8_t> result{mask.sizes()};
    forPixels(mask.sizes(), [&](int i, int j) {
      result(i, j) =
          forNeighbors(i, j, mask.sizes(), [&mask](int n, int m) { return mask(n, m) > 0; }) ? 255
                                                                                             : 0;
    });
    return result;
  }

  static auto dilate(const Mat<uint8_t> &mask) -> Mat<uint8_t> {
    Mat<uint8_t> result{mask.sizes()};
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

    auto i = begin(mask);
    auto j = begin(synthesizer.reference);
    auto k = begin(status);

    synthesizer.rasterizer.visit([&](const PixelValue<Vec3f> &x) {
      if (x.normDisp > 0) {
        const auto depthError = (x.depth() / *j - 1.F);

        if (abs(depthError) < m_maxDepthError) {
          if (*k != 0) {
            *i = 0;
          }
        } else if (m_ivSequenceParams.vme().vme_depth_low_quality_flag() && (depthError < 0.F)) {
          if (*k != 0) {
            *k = 0;
            *i = 255;
          }
        }
      }

      i++;
      j++;
      k++;

      return true;
    });
    for (int n = 0; n < m_erode; ++n) {
      mask = erode(mask);
    }
    for (int n = 0; n < m_dilate; ++n) {
      mask = dilate(mask);
    }
    synthesizer.maskAverage = float(accumulate(begin(mask), end(mask), 0)) /
                              (2.55F * float(mask.width() * mask.height()));
  }
};

HierarchicalPruner::HierarchicalPruner(const Json & /* unused */, const Json &nodeConfig)
    : m_impl(new Impl{nodeConfig}) {}

HierarchicalPruner::~HierarchicalPruner() = default;

void HierarchicalPruner::registerPruningRelation(MivBitstream::IvSequenceParams &ivSequenceParams,
                                                 const vector<bool> &isBasicView) {

  return m_impl->registerPruningRelation(ivSequenceParams, isBasicView);
}

auto HierarchicalPruner::prune(const MivBitstream::IvSequenceParams &ivSequenceParams,
                               const Common::MVD16Frame &views, const vector<bool> &isBasicView)
    -> Common::MaskList {
  return m_impl->prune(ivSequenceParams, views, isBasicView);
}
} // namespace TMIV::Pruner
