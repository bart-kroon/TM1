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

#include <TMIV/AtlasConstructor/HierarchicalPruner.h>

#include "PrunedMesh.h"
#include <TMIV/Metadata/DepthOccupancyTransform.h>
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
using namespace TMIV::Metadata;
using namespace TMIV::Renderer;
using namespace std;

namespace TMIV::AtlasConstructor {
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
  const AccumulatingPixel<Vec3f> m_config;
  bool m_intra{true};
  bool m_firstFrame{true};
  ViewParamsVector m_viewParamsVector;
  vector<bool> m_isBasicView;
  vector<unique_ptr<IncrementalSynthesizer>> m_synthesizers;
  vector<size_t> m_pruningOrder;
  vector<Frame<YUV400P8>> m_masks;

public:
  explicit Impl(const Json &nodeConfig)
      : m_maxDepthError{nodeConfig.require("maxDepthError").asFloat()},
        m_maxStretching{nodeConfig.require("maxStretching").asFloat()},
        m_erode{nodeConfig.require("erode").asInt()},
        m_dilate{nodeConfig.require("dilate").asInt()},
        m_config{nodeConfig.require("rayAngleParameter").asFloat(),
                 nodeConfig.require("depthParameter").asFloat(),
                 nodeConfig.require("stretchingParameter").asFloat(), m_maxStretching} {}

  auto prune(const Metadata::ViewParamsVector &viewParamsVector, const MVD16Frame &views,
             const vector<bool> &isBasicView) -> MaskList {
    m_intra = m_viewParamsVector != viewParamsVector || m_isBasicView != isBasicView;
    if (m_intra) {
      cout << "The pruning order is (re)determined at this frame\n";
      m_viewParamsVector = viewParamsVector;
      m_isBasicView = isBasicView;
    }
    prepareFrame(views);
    pruneFrame(views);

    m_firstFrame = false;
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
    transform(cbegin(m_viewParamsVector), cend(m_viewParamsVector), cbegin(views),
              back_inserter(m_masks),
              [](const ViewParams &viewParams, const TextureDepth16Frame &view) {
                auto mask = Frame<YUV400P8>{viewParams.size.x(), viewParams.size.y()};
                transform(cbegin(view.second.getPlane(0)), cend(view.second.getPlane(0)),
                          begin(mask.getPlane(0)), [ot = OccupancyTransform{viewParams}](auto x) {
                            // #94: When there are invalid pixels in a basic view, these should be
                            // excluded from the pruning mask
                            return uint8_t(ot.occupant(x) ? 255 : 0);
                          });
                return mask;
              });
  }

  void createSynthesizerPerPartialView(const MVD16Frame &views) {
    m_synthesizers.clear();
    for (size_t i = 0; i < m_viewParamsVector.size(); ++i) {
      if (!m_isBasicView[i]) {
        const auto depthTransform = DepthTransform<16>{m_viewParamsVector[i]};
        m_synthesizers.emplace_back(make_unique<IncrementalSynthesizer>(
            m_config, m_viewParamsVector[i].size, i, depthTransform.expandDepth(views[i].second)));
      }
    }
  }

  void synthesizeReferenceViews(const MVD16Frame &views) {
    if (m_synthesizers.empty()) {
      // Skip generation the meshes
      cout << "Nothing to prune: only basic views\n";
      return;
    }

    for (size_t i = 0; i < m_viewParamsVector.size(); ++i) {
      if (m_isBasicView[i]) {
        synthesizeViews(i, views[i]);
      }
    }
  }

  void pruneFrame(const MVD16Frame &views) {
    if (m_intra) {
      // Redetermine the pruning order on camera configuration changes...
      pruneIntraFrame(views);
    } else {
      // ... otherwise maintain the same pruning order
      pruneInterFrame(views);
    }

    auto sumValues = 0.;
    for (const auto &mask : m_masks) {
      sumValues = accumulate(begin(mask.getPlane(0)), end(mask.getPlane(0)), sumValues);
    }
    const auto lumaSamplesPerFrame = 2. * sumValues / 255e6;
    cout << "Non-pruned luma samples per frame is " << lumaSamplesPerFrame << "M\n";
  }

  void pruneIntraFrame(const MVD16Frame &views) {
    m_pruningOrder.clear();
    while (!m_synthesizers.empty()) {
      auto it = max_element(
          begin(m_synthesizers), end(m_synthesizers),
          [](const auto &s1, const auto &s2) { return s1->maskAverage < s2->maskAverage; });
      const auto i = (*it)->index;
      m_synthesizers.erase(it);
      synthesizeViews(i, views[i]);
      m_pruningOrder.push_back(i);
    }
  }

  void pruneInterFrame(const MVD16Frame &views) {
    for (auto i : m_pruningOrder) {
      auto it = find_if(begin(m_synthesizers), end(m_synthesizers),
                        [i](const auto &s) { return s->index == i; });
      m_synthesizers.erase(it);
      synthesizeViews(i, views[i]);
    }
  }

  // Synthesize the specified view to all remaining partial views.
  //
  // Special care is taken to make a pruned (masked) mesh once and re-use that
  // multiple times.
  void synthesizeViews(size_t index, const TextureDepth16Frame &view) {
    auto [ivertices, triangles, attributes] =
        unprojectPrunedView(view, m_viewParamsVector[index], m_masks[index].getPlane(0));

    if (m_isBasicView[index]) {
      cout << "Basic view ";
    } else {
      cout << "Prune view ";
    }

    const auto prec = cout.precision(2);
    const auto flags = cout.setf(ios::fixed, ios::floatfield);
    cout << setw(2) << index << " (" << setw(3) << m_viewParamsVector[index].name
         << "): " << ivertices.size() << " vertices ("
         << 100. * double(ivertices.size()) / (view.first.getWidth() * view.first.getHeight())
         << "% of full view)\n";
    cout.precision(prec);
    cout.setf(flags);

    for (auto &s : m_synthesizers) {
      auto overtices = project(ivertices, m_viewParamsVector[index], m_viewParamsVector[s->index]);
      weightedSphere(m_viewParamsVector[s->index], overtices, triangles);
      s->rasterizer.submit(overtices, attributes, triangles);
      s->rasterizer.run();
      updateMask(*s);
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

  static auto erode(Mat<uint8_t> &mask) -> Mat<uint8_t> {
    Mat<uint8_t> result{mask.sizes()};
    forPixels(mask.sizes(), [&](int i, int j) {
      result(i, j) =
          forNeighbors(i, j, mask.sizes(), [&mask](int n, int m) { return mask(n, m) > 0; }) ? 255
                                                                                             : 0;
    });
    return result;
  }

  static auto dilate(Mat<uint8_t> &mask) -> Mat<uint8_t> {
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
    auto i = begin(mask);
    auto j = begin(synthesizer.reference);
    synthesizer.rasterizer.visit([&](const PixelValue<Vec3f> &x) {
      const auto depthError = abs(x.depth() / *j++ - 1.F);
      *i++ = uint8_t(x.normDisp > 0 && depthError < m_maxDepthError ? 0 : 255);
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

auto HierarchicalPruner::prune(const Metadata::ViewParamsVector &viewParamsVector,
                               const Common::MVD16Frame &views,
                               const std::vector<bool> &isBasicView) -> Common::MaskList {
  return m_impl->prune(viewParamsVector, views, isBasicView);
}
} // namespace TMIV::AtlasConstructor
