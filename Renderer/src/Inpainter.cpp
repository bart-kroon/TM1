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

#include <TMIV/Renderer/Inpainter.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::Renderer {
namespace {
template <typename YUVD>
void perform2WayInpainting(
    YUVD &yuvd, const double &DepthBlendingThreshold,
    int inpaintingType /*0 for horizontal, 1 for vertical, 2 for omni*/,
    const Common::Mat<int> &nonEmptyNeighbor1,
    const Common::Mat<int> &nonEmptyNeighbor2,
    const Common::Mat<int> &mapERP2Cassini = Common::Mat<int>()) {

  auto &Y = yuvd.first.getPlane(0);
  auto &U = yuvd.first.getPlane(1);
  auto &V = yuvd.first.getPlane(2);
  auto &D = yuvd.second.getPlane(0);

  const int width = int(Y.width());
  const int height = int(Y.height());

  for (int h = 0, pp = 0; h < height; h++) {
    for (int w = 0; w < width; w++, pp++) {

      if (D(h, w) != 0) {
        continue;
      }

      bool use1 = false;
      bool use2 = false;

      int w0, h0, w1, h1, w2, h2;

      if (inpaintingType == 2) { // omnidirectional
        bool pointExistsInCassini = mapERP2Cassini(h, w) != -1;
        if (!pointExistsInCassini) {
          continue;
        }

        h0 = mapERP2Cassini(h, w) / width; // current pixel in Cassini
                                           // projection
        w0 = mapERP2Cassini(h, w) % width;

        h1 = nonEmptyNeighbor1(h0, w0) /
             width; // left neighbor in Cassini projection
        w1 = nonEmptyNeighbor1(h0, w0) % width;

        h2 = nonEmptyNeighbor2(h0, w0) /
             width; // right neighbor in Cassini projection
        w2 = nonEmptyNeighbor2(h0, w0) % width;
      } else {  // perspective
        h0 = h; // current pixel
        w0 = w;

        h1 = inpaintingType ? h0
                            : nonEmptyNeighbor1(h0, w0); // left or top neighbor
        w1 = inpaintingType ? nonEmptyNeighbor1(h0, w0) : w0;

        h2 = inpaintingType
                 ? h0
                 : nonEmptyNeighbor2(h0, w0); // right or bottom neighbor
        w2 = inpaintingType ? nonEmptyNeighbor2(h0, w0) : w0;
      }

      if (nonEmptyNeighbor1(h0, w0) != -1) {
        if (nonEmptyNeighbor2(h0, w0) != -1) {

          float farthestDepth = D(h1, w1) < D(h2, w2) ? D(h1, w1) : D(h2, w2);
          if (D(h1, w1) - farthestDepth <= DepthBlendingThreshold) {
            use1 = true;
          }
          if (D(h2, w2) - farthestDepth <= DepthBlendingThreshold) {
            use2 = true;
          }
        } else {
          use1 = true;
        }
      } else {
        if (nonEmptyNeighbor2(h0, w0) != -1) {
          use2 = true;
        } else {
          continue;
        }
      }

      if (use1) {
        if (use2) {

          auto dist1 = sqrt(
              static_cast<float>((h - h1) * (h - h1) + (w - w1) * (w - w1)));
          auto dist2 = sqrt(
              static_cast<float>((h - h2) * (h - h2) + (w - w2) * (w - w2)));
          float sumdist = dist1 + dist2;
          float weight1 = dist2 / sumdist;
          float weight2 = dist1 / sumdist;

          Y(h, w) = static_cast<int>(Y(h1, w1) * weight1);
          U(h, w) = static_cast<int>(U(h1, w1) * weight1);
          V(h, w) = static_cast<int>(V(h1, w1) * weight1);
          D(h, w) = static_cast<int>(D(h1, w1) * weight1);

          Y(h, w) += static_cast<int>(Y(h2, w2) * weight2);
          U(h, w) += static_cast<int>(U(h2, w2) * weight2);
          V(h, w) += static_cast<int>(V(h2, w2) * weight2);
          D(h, w) += static_cast<int>(D(h2, w2) * weight2);
        } else {
          Y(h, w) = Y(h1, w1);
          U(h, w) = U(h1, w1);
          V(h, w) = V(h1, w1);
          D(h, w) = D(h1, w1);
        }
      } else {
        if (use2) {
          Y(h, w) = Y(h2, w2);
          U(h, w) = U(h2, w2);
          V(h, w) = V(h2, w2);
          D(h, w) = D(h2, w2);
        } else {
          continue;
        }
      }

    } // w
  }   // h
}

template <typename YUVD> void fillVerticalCracks(YUVD &yuvd) {

  auto &Y = yuvd.first.getPlane(0);
  auto &U = yuvd.first.getPlane(1);
  auto &V = yuvd.first.getPlane(2);
  auto &D = yuvd.second.getPlane(0);

  const int width = int(Y.width());
  const int height = int(Y.height());

  // fill vertical cracks
  for (int h = 0; h < height; h++) {
    for (int w = 1; w < width - 1; w++) {
      if (D(h, w) == 0 && D(h, w - 1) != 0 && D(h, w + 1) != 0) {
        Y(h, w) = (Y(h, w - 1) + Y(h, w + 1)) / 2;
        U(h, w) = (U(h, w - 1) + U(h, w + 1)) / 2;
        V(h, w) = (V(h, w - 1) + V(h, w + 1)) / 2;
        D(h, w) = (D(h, w - 1) + D(h, w + 1)) / 2;
      }
    }
  }
}

template <typename YUVD>
void inpaintOmnidirectionalView(YUVD &yuvd,
                                const double &DepthBlendingThreshold,
                                const double &angleRange) {

  auto &Y = yuvd.first.getPlane(0);
  auto &D = yuvd.second.getPlane(0);

  const int width = int(Y.width());
  const int height = int(Y.height());

  Common::Mat<int> isHole;
  isHole.resize(height, width);

  Common::Mat<int> nonEmptyNeighborL;
  nonEmptyNeighborL.resize(height, width);

  Common::Mat<int> nonEmptyNeighborR;
  nonEmptyNeighborR.resize(height, width);

  Common::Mat<int> mapERP2Cassini;
  mapERP2Cassini.resize(height, width);

  Common::Mat<int> mapCassini2ERP;
  mapCassini2ERP.resize(height, width);

  for (int h = 0; h < height; h++) {
    for (int w = 0; w < width; w++) {
      nonEmptyNeighborL(h, w) = -1;
      nonEmptyNeighborR(h, w) = -1;
      mapERP2Cassini(h, w) = -1;
      mapCassini2ERP(h, w) = -1;
      isHole(h, w) = 1;
    }
  }

  int width2 = width / 2;
  int height2 = height / 2;
  double tmpH, tmpW;

  int oldPP, oldH, oldW;
  int newPP;
  double newH, newW;
  int iNewH, iNewW;

  for (int h = 0; h < height; h++) {
    oldH = h - height2;
    for (int w = 0; w < width; w++) {
      oldPP = h * width + w;

      oldW = w - width2;
      tmpH = sqrt(height * h - h * h);
      if (tmpH / height2 > angleRange) {
        tmpH = height2 * angleRange;
      }
      newW = oldW * tmpH / height2;
      newW += width2;

      tmpW = sqrt(width * newW - newW * newW);
      newH = oldH * width2 / tmpW;
      newH += height2;

      iNewH = int(newH + 0.5);
      iNewW = int(newW + 0.5);

      if (iNewH < 0 || iNewH >= height) {
        continue;
      }

      newPP = iNewH * width + iNewW;

      mapERP2Cassini(h, w) = newPP;
      if (isHole(iNewH, iNewW) == 1) {
        mapCassini2ERP(iNewH, iNewW) = oldPP;
      }
      if (D(h, w) != 0) {
        isHole(iNewH, iNewW) = 0; // 1 if hole
      }
    }
  }

  // analysis from top-left

  for (int h = 0; h < height; h++) {
    for (int w = 0; w < width; w++) {

      nonEmptyNeighborL(h, w) = mapCassini2ERP(h, w);
      if (isHole(h, w)) {
        if (w > 0) {
          nonEmptyNeighborL(h, w) = nonEmptyNeighborL(h, w - 1);
        } else {
          nonEmptyNeighborL(h, w) = -1;
        }
      }

    } // w
  }   // h

  // analysis from bottom-right

  for (int h = height - 1; h >= 0; h--) {
    for (int w = width - 1; w >= 0; w--) {

      nonEmptyNeighborR(h, w) = mapCassini2ERP(h, w);
      if (isHole(h, w)) {
        if (w < width - 1) {
          nonEmptyNeighborR(h, w) = nonEmptyNeighborR(h, w + 1);
        } else {
          nonEmptyNeighborR(h, w) = -1;
        }
      }

    } // w
  }   // h

  // inpainting

  perform2WayInpainting(yuvd, DepthBlendingThreshold, 2, nonEmptyNeighborL,
                        nonEmptyNeighborR, mapERP2Cassini);
}

template <typename YUVD>
void inpaintPerspectiveView(YUVD &yuvd, const double &DepthBlendingThreshold) {

  auto &D = yuvd.second.getPlane(0);

  const int width = int(D.width());
  const int height = int(D.height());

  Common::Mat<int> nonEmptyNeighborL;
  nonEmptyNeighborL.resize(height, width);

  Common::Mat<int> nonEmptyNeighborR;
  nonEmptyNeighborR.resize(height, width);

  Common::Mat<int> nonEmptyNeighborT;
  nonEmptyNeighborT.resize(height, width);

  Common::Mat<int> nonEmptyNeighborB;
  nonEmptyNeighborB.resize(height, width);

  // analysis from top-left

  for (int h = 0; h < height; h++) {
    for (int w = 0; w < width; w++) {

      nonEmptyNeighborL(h, w) = w;

      if (D(h, w) == 0) {
        if (w > 0) {
          nonEmptyNeighborL(h, w) = nonEmptyNeighborL(h, w - 1);
        } else {
          nonEmptyNeighborL(h, w) = -1;
        }
      }

    } // w
  }   // h

  // analysis from bottom-right

  for (int h = height - 1; h >= 0; h--) {
    for (int w = width - 1; w >= 0; w--) {

      nonEmptyNeighborR(h, w) = w;

      if (D(h, w) == 0) {
        if (w < width - 1) {
          nonEmptyNeighborR(h, w) = nonEmptyNeighborR(h, w + 1);
        } else {
          nonEmptyNeighborR(h, w) = -1;
        }
      }

    } // w
  }   // h

  // horizontal inpainting

  perform2WayInpainting(yuvd, DepthBlendingThreshold, 1, nonEmptyNeighborL,
                        nonEmptyNeighborR);

  // analysis from top-left

  for (int h = 0; h < height; h++) {
    for (int w = 0; w < width; w++) {

      nonEmptyNeighborT(h, w) = h;

      if (D(h, w) == 0) {
        if (h > 0) {
          nonEmptyNeighborT(h, w) = nonEmptyNeighborT(h - 1, w);
        } else {
          nonEmptyNeighborT(h, w) = -1;
        }
      }

    } // w
  }   // h

  // analysis from bottom-right

  for (int h = height - 1; h >= 0; h--) {
    for (int w = width - 1; w >= 0; w--) {

      nonEmptyNeighborB(h, w) = h;

      if (D(h, w) == 0) {
        if (h < height - 1) {
          nonEmptyNeighborB(h, w) = nonEmptyNeighborB(h, w + 1);
        } else {
          nonEmptyNeighborB(h, w) = -1;
        }
      }

    } // w
  }   // h

  // vertical inpainting

  perform2WayInpainting(yuvd, DepthBlendingThreshold, 0, nonEmptyNeighborT,
                        nonEmptyNeighborB);
}

template <typename YUVD>
void inplaceInpaint_impl(YUVD &yuvd, const CameraParameters &meta) {
  static_assert(std::is_same_v<YUVD, Texture444Depth10Frame> ||
                std::is_same_v<YUVD, Texture444Depth16Frame>);

  double DepthBlendingThreshold = 2.56; // 1% of bit depth

  if (std::is_same_v<YUVD, Texture444Depth10Frame>) {
    DepthBlendingThreshold = 1024.0 / 100;
  }
  if (std::is_same_v<YUVD, Texture444Depth16Frame>) {
    DepthBlendingThreshold = 65536.0 / 100;
  }

  fillVerticalCracks(yuvd);

  if (meta.type == ProjectionType::ERP) {
    double angleRange = (meta.erpPhiRange[1] - meta.erpPhiRange[0]) / M_2PI;
    inpaintOmnidirectionalView(yuvd, DepthBlendingThreshold, angleRange);
  }

  inpaintPerspectiveView(yuvd, DepthBlendingThreshold);
}
} // namespace

Inpainter::Inpainter(const Json & /*rootNode*/,
                     const Json & /*componentNode*/) {}

void Inpainter::inplaceInpaint(Texture444Depth10Frame &viewport,
                               const CameraParameters &metadata) const {
  inplaceInpaint_impl(viewport, metadata);
}

void Inpainter::inplaceInpaint(Texture444Depth16Frame &viewport,
                               const CameraParameters &metadata) const {
  inplaceInpaint_impl(viewport, metadata);
}
} // namespace TMIV::Renderer
