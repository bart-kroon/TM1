/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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

#include <TMIV/AtlasConstructor/Pruner.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/Image/Image.h>
#include <TMIV/Renderer/reprojectPoints.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Image;
using namespace TMIV::Renderer;

namespace TMIV::AtlasConstructor {

Pruner::Pruner(const Common::Json & /*rootNode*/,
               const Common::Json &componentNode) {

  if (auto subnode = componentNode.optional("RedundancyFactor"))
    m_redundancyFactor = subnode.asFloat();

  if (auto subnode = componentNode.optional("ErosionIter"))
    m_erosionIter = subnode.asInt();

  if (auto subnode = componentNode.optional("DilationIter"))
    m_dilationIter = subnode.asInt();

  if (auto subnode = componentNode.optional("MaxAdditionalView"))
    m_maxAdditionalView = subnode.asInt();
}

MaskList Pruner::prune(const Metadata::CameraParameterList &cameras,
                       const MVD16Frame &views,
                       const std::vector<std::uint8_t> &shouldNotBePruned) {

  // Sort cameras for pruning
  std::vector<int> cameraOrderId(cameras.size());

  std::iota(cameraOrderId.begin(), cameraOrderId.end(), 0);

  std::sort(cameraOrderId.begin(), cameraOrderId.end(),
            [&shouldNotBePruned](int i1, int i2) {
              if (shouldNotBePruned[i1] != shouldNotBePruned[i2])
                return (shouldNotBePruned[i1] != 0);
              else
                return (i1 < i2);
            });

  // Possible discard some additional views (for debugging purpose)
  int maxView = std::min(
      int(cameraOrderId.size()),
      int(std::count(shouldNotBePruned.begin(), shouldNotBePruned.end(), 1) +
          m_maxAdditionalView));

  // Pruning loop
  int nbView = (int)views.size();
  MaskList masks(nbView);
  std::vector<Mat<float>> depthMapExpanded(nbView);

  for (int id1 = 0; id1 < nbView; id1++) {
    int viewToPruneId = cameraOrderId[id1];
    auto &maskToPrune = masks[viewToPruneId];

    maskToPrune.resize(views[viewToPruneId].first.getWidth(),
                       views[viewToPruneId].first.getHeight());
    auto &bufferToPrune = maskToPrune.getPlane(0);

    if (id1 < maxView) {
      std::fill(bufferToPrune.begin(), bufferToPrune.end(), uint8_t(255));
      depthMapExpanded[viewToPruneId] =
          expandDepth(cameras[viewToPruneId], views[viewToPruneId].second);

      if (!shouldNotBePruned[viewToPruneId]) {
        // Depth-based redundancy removal
        const Mat<float> &depthMapToPrune = depthMapExpanded[viewToPruneId];
        Mat<Vec2f> gridMapToPrune = imagePositions(cameras[viewToPruneId]);

        for (int id2 = 0; id2 < id1; id2++) {
          int viewPrunedId = cameraOrderId[id2];
          const Mat<float> &depthMapPruned = depthMapExpanded[viewPrunedId];

          auto ptsToPruneOnPruned =
              reprojectPoints(cameras[viewToPruneId], cameras[viewPrunedId],
                              gridMapToPrune, depthMapToPrune);
          int lastXPruned = cameras[viewPrunedId].size.x() - 1,
              lastYPruned = cameras[viewPrunedId].size.y() - 1;

          for (auto k = 0u; k < bufferToPrune.size(); k++) {
            auto &mask = bufferToPrune[k];

            if (0 < mask) {
              float zToPrune = depthMapToPrune[k];

              if (!std::isnan(zToPrune)) {
                float zToPruneOnPruned = ptsToPruneOnPruned.second[k];

                if (!std::isnan(zToPruneOnPruned)) {
                  const Vec2f &xyToPruneOnPruned = ptsToPruneOnPruned.first[k];

                  int x1 = std::max(0, int(floor(xyToPruneOnPruned.x()))),
                      x2 = std::min(lastXPruned,
                                    int(ceil(xyToPruneOnPruned.x())));
                  int y1 = std::max(0, int(floor(xyToPruneOnPruned.y()))),
                      y2 = std::min(lastYPruned,
                                    int(ceil(xyToPruneOnPruned.y())));

                  for (int y = y1; y <= y2; y++) {
                    for (int x = x1; x <= x2; x++) {
                      float zPruned = depthMapPruned(y, x);

                      if (!std::isnan(zPruned)) {
                        if ((fabs(zToPruneOnPruned - zPruned) <
                             m_redundancyFactor *
                                 std::min(zPruned, zToPruneOnPruned))) {
                          mask = 0;
                          goto endloop;
                        }
                      }
                    }
                  }
                }
              } else
                mask = 0;
            }

          endloop:;
          }
        }

        // Mask post-processing
        Mask maskPostProc(views[viewToPruneId].first.getWidth(),
                          views[viewToPruneId].first.getHeight());
        auto &bufferPostProc = maskPostProc.getPlane(0);

        int w = bufferToPrune.width(), h = bufferToPrune.height();
        int wLast = w - 1, hLast = h - 1;
        std::array<int, 8> neighbourOffset = {-1 - w, -w,     1 - w, -1,
                                              1,      -1 + w, w,     1 + w};

        // Erosion
        if (0 < m_erosionIter) {
          auto &inputBuffer =
              (m_erosionIter % 2) ? bufferToPrune : bufferPostProc;
          auto &outputBuffer =
              (m_erosionIter % 2) ? bufferPostProc : bufferToPrune;

          inputBuffer = bufferToPrune;

          for (int erosionId = 0; erosionId < m_erosionIter; erosionId++) {
            for (int y = 1, k1 = w + 1; y < hLast; y++, k1 += w) {
              for (int x = 1, k2 = k1; x < wLast; x++, k2++) {
                auto maskIn = inputBuffer[k2];
                auto &maskOut = outputBuffer[k2];

                maskOut = maskIn;

                if (0 < maskIn) {
                  for (auto o : neighbourOffset) {
                    if (inputBuffer[k2 + o] == 0) {
                      maskOut = 0;
                      break;
                    }
                  }
                }
              }
            }

            std::swap(inputBuffer, outputBuffer);
          }
        }

        // Dilation
        if (0 < m_dilationIter) {
          auto &inputBuffer =
              (m_erosionIter % 2) ? bufferToPrune : bufferPostProc;
          auto &outputBuffer =
              (m_erosionIter % 2) ? bufferPostProc : bufferToPrune;

          inputBuffer = bufferToPrune;

          for (int dilationId = 0; dilationId < m_dilationIter; dilationId++) {
            for (int y = 1, k1 = w + 1; y < hLast; y++, k1 += w) {
              for (int x = 1, k2 = k1; x < wLast; x++, k2++) {
                auto maskIn = inputBuffer[k2];
                auto &maskOut = outputBuffer[k2];

                maskOut = maskIn;

                if (0 == maskIn) {
                  for (auto o : neighbourOffset) {
                    if (0 < inputBuffer[k2 + o]) {
                      maskOut = 1;
                      break;
                    }
                  }
                }
              }
            }

            std::swap(inputBuffer, outputBuffer);
          }
        }
      }
    } else
      std::fill(bufferToPrune.begin(), bufferToPrune.end(), uint8_t(0));
  }

  return masks;
}

} // namespace TMIV::AtlasConstructor
