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

#include <TMIV/Renderer/Inpainter.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::Renderer {
namespace {
template <typename YUVD>
void inplaceInpaint_impl(YUVD &yuvd, const CameraParameters &meta) {
  static_assert(std::is_same_v<YUVD, Texture444Depth10Frame> ||
                std::is_same_v<YUVD, Texture444Depth16Frame>);

  auto &Y = yuvd.first.getPlane(0);
  auto &U = yuvd.first.getPlane(1);
  auto &V = yuvd.first.getPlane(2);
  auto &D = yuvd.second.getPlane(0);

  const int width = int(Y.width());
  const int height = int(Y.height());

  const int imsize = width * height;

  double DepthBlendingThreshold = 1;

  if (meta.type == ProjectionType::ERP) {
    double angleRange = (meta.erpPhiRange[1] - meta.erpPhiRange[0]) / M_2PI;

    bool *isHole = new bool[imsize];

    int **nonEmptyNeighborL = new int *[2];
    nonEmptyNeighborL[0] = new int[imsize];
    nonEmptyNeighborL[1] = new int[imsize];

    int **nonEmptyNeighborR = new int *[2];
    nonEmptyNeighborR[0] = new int[imsize];
    nonEmptyNeighborR[1] = new int[imsize];

    int **mapERP2Cassini = new int *[2];
    mapERP2Cassini[0] = new int[imsize];
    mapERP2Cassini[1] = new int[imsize];

    int **mapCassini2ERP = new int *[2];
    mapCassini2ERP[0] = new int[imsize];
    mapCassini2ERP[1] = new int[imsize];

    for (int i = 0; i < 2; i++) {
      for (int pp = 0; pp < imsize; pp++) {
        nonEmptyNeighborL[i][pp] = -1;
        nonEmptyNeighborR[i][pp] = -1;
        mapERP2Cassini[i][pp] = -1;
        mapCassini2ERP[i][pp] = -1;
      }
    }
    for (int pp = 0; pp < imsize; pp++)
      isHole[pp] = 1;

    int width2 = width / 2;
    int height2 = height / 2;
    double tmpH, tmpW;

    int oldPP, oldH, oldW;
    int newPP;
    double newH, newW;

    for (int h = 0; h < height; h++) {
      oldH = h - height2;
      for (int w = 0; w < width; w++) {
        oldPP = h * width + w;

        oldW = w - width2;
        tmpH = sqrt(height * h - h * h);
        if (tmpH / height2 > angleRange)
          tmpH = height2 * angleRange;
        newW = oldW * tmpH / height2;
        newW += width2;

        tmpW = sqrt(width * newW - newW * newW);
        newH = oldH * width2 / tmpW;
        newH += height2;

        if (int(newH + 0.5) < 0 || int(newH + 0.5) >= height)
          continue;

        newPP = int(newH + 0.5) * width + int(newW + 0.5);

        mapERP2Cassini[0][oldPP] = int(newH + 0.5);
        mapERP2Cassini[1][oldPP] = int(newW + 0.5);

        if (isHole[newPP] == 1) {
          mapCassini2ERP[0][newPP] = h;
          mapCassini2ERP[1][newPP] = w;
        }

        if (D(h, w) != 0)
          isHole[newPP] = 0; // 1 if hole
      }
    }

    // analysis from top-left

    for (int h = 0; h < height; h++) {
      int hW = h * width;
      for (int w = 0; w < width; w++) {
        int pp = hW + w;

        nonEmptyNeighborL[0][pp] = mapCassini2ERP[0][pp];
        nonEmptyNeighborL[1][pp] = mapCassini2ERP[1][pp];

        if (isHole[pp]) {
          if (w > 0) {
            nonEmptyNeighborL[0][pp] = nonEmptyNeighborL[0][pp - 1];
            nonEmptyNeighborL[1][pp] = nonEmptyNeighborL[1][pp - 1];
          } else {
            nonEmptyNeighborL[0][pp] = -1;
            nonEmptyNeighborL[1][pp] = -1;
          }
        }

      } // w
    }   // h

    // analysis from bottom-right

    for (int h = height - 1; h >= 0; h--) {
      int hW = h * width;
      for (int w = width - 1; w >= 0; w--) {
        int pp = hW + w;

        nonEmptyNeighborR[0][pp] = mapCassini2ERP[0][pp];
        nonEmptyNeighborR[1][pp] = mapCassini2ERP[1][pp];

        if (isHole[pp]) {
          if (w < width - 1) {
            nonEmptyNeighborR[0][pp] = nonEmptyNeighborR[0][pp + 1];
            nonEmptyNeighborR[1][pp] = nonEmptyNeighborR[1][pp + 1];
          } else {
            nonEmptyNeighborR[0][pp] = -1;
            nonEmptyNeighborR[1][pp] = -1;
          }
        }

      } // w
    }   // h

    // inpainting
    double distL, distR, sumdist, weightL, weightR;

    for (int h = 0, pp = 0; h < height; h++) {
      for (int w = 0; w < width; w++, pp++) {

        int posInCassini =
            mapERP2Cassini[0][pp] * width + mapERP2Cassini[1][pp];
        bool pointExistsInCassini = mapERP2Cassini[0][pp] != -1;

        if (D(h, w) == 0 && pointExistsInCassini) {

          int rowOfLeftInCassini = nonEmptyNeighborL[0][posInCassini];
          int rowOfRightInCassini = nonEmptyNeighborR[0][posInCassini];

          int colOfLeftInCassini = nonEmptyNeighborL[1][posInCassini];
          int colOfRightInCassini = nonEmptyNeighborR[1][posInCassini];

          int posOfLeftInCassini =
              rowOfLeftInCassini * width + colOfLeftInCassini;
          int posOfRightInCassini =
              rowOfRightInCassini * width + colOfRightInCassini;

          bool leftInCassiniExists = (nonEmptyNeighborL[0][posInCassini] != -1);
          bool rightInCassiniExists =
              (nonEmptyNeighborR[0][posInCassini] != -1);

          if (leftInCassiniExists && rightInCassiniExists) {
            if (D(rowOfLeftInCassini, colOfLeftInCassini) >
                D(rowOfRightInCassini, colOfRightInCassini) +
                    DepthBlendingThreshold) { // right further
              Y(h, w) = Y(rowOfRightInCassini, colOfRightInCassini);
              U(h, w) = U(rowOfRightInCassini, colOfRightInCassini);
              V(h, w) = V(rowOfRightInCassini, colOfRightInCassini);
              D(h, w) = D(rowOfRightInCassini, colOfRightInCassini);
            } else if (D(rowOfRightInCassini, colOfRightInCassini) >
                       D(rowOfLeftInCassini, colOfLeftInCassini) +
                           DepthBlendingThreshold) { // left further
              Y(h, w) = Y(rowOfLeftInCassini, colOfLeftInCassini);
              U(h, w) = U(rowOfLeftInCassini, colOfLeftInCassini);
              V(h, w) = V(rowOfLeftInCassini, colOfLeftInCassini);
              D(h, w) = D(rowOfLeftInCassini, colOfLeftInCassini);
            } else { // blend
              distL = sqrt((h - rowOfLeftInCassini) * (h - rowOfLeftInCassini) +
                           (w - colOfLeftInCassini) * (w - colOfLeftInCassini));
              distR =
                  sqrt((h - rowOfRightInCassini) * (h - rowOfRightInCassini) +
                       (w - colOfRightInCassini) * (w - colOfRightInCassini));
              sumdist = distL + distR;
              weightL = distR / sumdist;
              weightR = distL / sumdist;

              Y(h, w) = Y(rowOfLeftInCassini, colOfLeftInCassini) * weightL +
                        Y(rowOfRightInCassini, colOfRightInCassini) * weightR;
              U(h, w) = Y(rowOfLeftInCassini, colOfLeftInCassini) * weightL +
                        U(rowOfRightInCassini, colOfRightInCassini) * weightR;
              V(h, w) = Y(rowOfLeftInCassini, colOfLeftInCassini) * weightL +
                        V(rowOfRightInCassini, colOfRightInCassini) * weightR;
              D(h, w) = Y(rowOfLeftInCassini, colOfLeftInCassini) * weightL +
                        D(rowOfRightInCassini, colOfRightInCassini) * weightR;
            }
          } else if (leftInCassiniExists) {
            Y(h, w) = Y(rowOfLeftInCassini, colOfLeftInCassini);
            U(h, w) = U(rowOfLeftInCassini, colOfLeftInCassini);
            V(h, w) = V(rowOfLeftInCassini, colOfLeftInCassini);
            D(h, w) = D(rowOfLeftInCassini, colOfLeftInCassini);
          } else if (rightInCassiniExists) {
            Y(h, w) = Y(rowOfRightInCassini, colOfRightInCassini);

            U(h, w) = U(rowOfRightInCassini, colOfRightInCassini);
            V(h, w) = V(rowOfRightInCassini, colOfRightInCassini);
            D(h, w) = D(rowOfRightInCassini, colOfRightInCassini);
          }
        }
      }
    }

    // remaining points
    for (int h = 0; h < height; h++) {
      int hW = h * width;
      for (int w = 0; w < width; w++) {
        int pp = hW + w;
        if (D(h, w) == 0) {
          if (w > 0) {
            Y(h, w) = Y(h, w - 1);
            U(h, w) = U(h, w - 1);
            V(h, w) = V(h, w - 1);
            D(h, w) = D(h, w - 1);
          } else if (h > 0) {
            Y(h, w) = Y(h - 1, w);
            U(h, w) = U(h - 1, w);
            V(h, w) = V(h - 1, w);
            D(h, w) = D(h - 1, w);
          }
        }
      }
    }

    for (int h = height - 1; h >= 0; h--) {
      int hW = h * width;
      for (int w = width - 1; w >= 0; w--) {
        int pp = hW + w;
        if (D(h, w) == 0) {
          if (w < width - 1) {
            Y(h, w) = Y(h, w + 1);
            U(h, w) = U(h, w + 1);
            V(h, w) = V(h, w + 1);
            D(h, w) = D(h, w + 1);
          } else if (h < height - 1) {
            Y(h, w) = Y(h + 1, w);
            U(h, w) = U(h + 1, w);
            V(h, w) = V(h + 1, w);
            D(h, w) = D(h + 1, w);
          }
        }
      }
    }

    delete nonEmptyNeighborL[0];
    delete nonEmptyNeighborL[1];
    delete nonEmptyNeighborL;

    delete nonEmptyNeighborR[0];
    delete nonEmptyNeighborR[1];
    delete nonEmptyNeighborR;

    delete mapERP2Cassini[0];
    delete mapERP2Cassini[1];
    delete mapERP2Cassini;

    delete mapCassini2ERP[0];
    delete mapCassini2ERP[1];
    delete mapCassini2ERP;

    delete isHole;

  } else {
    int *nonEmptyNeighborL = new int[imsize];
    int *nonEmptyNeighborR = new int[imsize];
    int *nonEmptyNeighborT = new int[imsize];
    int *nonEmptyNeighborB = new int[imsize];

    // analysis from top-left

    for (int h = 0; h < height; h++) {
      int hW = h * width;
      for (int w = 0; w < width; w++) {
        int pp = hW + w;

        nonEmptyNeighborL[pp] = w;

        if (D(h, w) == 0) {

          if (w > 0)
            nonEmptyNeighborL[pp] = nonEmptyNeighborL[pp - 1];
          else
            nonEmptyNeighborL[pp] = -1;
        }

      } // w
    }   // h

    // analysis from bottom-right

    for (int h = height - 1; h >= 0; h--) {
      int hW = h * width;
      for (int w = width - 1; w >= 0; w--) {
        int pp = hW + w;

        nonEmptyNeighborR[pp] = w;

        if (D(h, w) == 0) {

          if (w < width - 1)
            nonEmptyNeighborR[pp] = nonEmptyNeighborR[pp + 1];
          else
            nonEmptyNeighborR[pp] = -1;
        }

      } // w
    }   // h

    // horizontal inpainting

    for (int h = 0, pp = 0; h < height; h++) {
      for (int w = 0; w < width; w++, pp++) {

        if (D(h, w) != 0)
          continue;

        int dist;
        float weight;

        bool useL = false;
        bool useR = false;

        if (nonEmptyNeighborL[pp] != -1) {
          if (nonEmptyNeighborR[pp] != -1) {

            float farthestDepth =
                D(h, nonEmptyNeighborL[pp]) < D(h, nonEmptyNeighborR[pp])
                    ? D(h, nonEmptyNeighborL[pp])
                    : D(h, nonEmptyNeighborR[pp]);
            if (D(h, nonEmptyNeighborL[pp]) - farthestDepth <=
                DepthBlendingThreshold)
              useL = true;
            if (D(h, nonEmptyNeighborR[pp]) - farthestDepth <=
                DepthBlendingThreshold)
              useR = true;
          } else {
            useL = true;
          }
        } else {
          if (nonEmptyNeighborR[pp] != -1) {
            useR = true;
          } else {
            continue;
          }
        }

        if (useL) {
          if (useR) {
            float weightL = 1.0 / abs(w - nonEmptyNeighborL[pp]);
            float weightR = 1.0 / abs(w - nonEmptyNeighborR[pp]);
            float sumWeights = weightL + weightR;

            weightL /= sumWeights;
            weightR /= sumWeights;

            Y(h, w) = Y(h, nonEmptyNeighborL[pp]) * weightL;
            U(h, w) = U(h, nonEmptyNeighborL[pp]) * weightL;
            V(h, w) = V(h, nonEmptyNeighborL[pp]) * weightL;
            D(h, w) = D(h, nonEmptyNeighborL[pp]) * weightL;

            Y(h, w) += (Y(h, nonEmptyNeighborR[pp]) * weightR);
            U(h, w) += (U(h, nonEmptyNeighborR[pp]) * weightR);
            V(h, w) += (V(h, nonEmptyNeighborR[pp]) * weightR);
            D(h, w) += (D(h, nonEmptyNeighborR[pp]) * weightR);
          } else {
            Y(h, w) = Y(h, nonEmptyNeighborL[pp]);
            U(h, w) = U(h, nonEmptyNeighborL[pp]);
            V(h, w) = V(h, nonEmptyNeighborL[pp]);
            D(h, w) = D(h, nonEmptyNeighborL[pp]);
          }
        } else {
          if (useR) {
            Y(h, w) = Y(h, nonEmptyNeighborR[pp]);
            U(h, w) = U(h, nonEmptyNeighborR[pp]);
            V(h, w) = V(h, nonEmptyNeighborR[pp]);
            D(h, w) = D(h, nonEmptyNeighborR[pp]);
          } else {
            continue;
          }
        }

      } // w
    }   // h

    // analysis from top-left

    for (int h = 0; h < height; h++) {
      int hW = h * width;
      for (int w = 0; w < width; w++) {
        int pp = hW + w;

        nonEmptyNeighborT[pp] = h;

        if (D(h, w) == 0) {

          if (h > 0)
            nonEmptyNeighborT[pp] = nonEmptyNeighborT[pp - width];
          else
            nonEmptyNeighborT[pp] = -1;
        }

      } // w
    }   // h

    // analysis from bottom-right

    for (int h = height - 1; h >= 0; h--) {
      int hW = h * width;
      for (int w = width - 1; w >= 0; w--) {
        int pp = hW + w;

        nonEmptyNeighborB[pp] = h;

        if (D(h, w) == 0) {

          if (h < height - 1)
            nonEmptyNeighborB[pp] = nonEmptyNeighborB[pp + 1];
          else
            nonEmptyNeighborB[pp] = -1;
        }

      } // w
    }   // h

    // vertical inpainting

    for (int h = 0, pp = 0; h < height; h++) {
      for (int w = 0; w < width; w++, pp++) {

        if (D(h, w) != 0)
          continue;

        int dist;
        float weight;

        bool useT = false;
        bool useB = false;

        if (nonEmptyNeighborT[pp] != -1) {
          if (nonEmptyNeighborB[pp] != -1) {

            float farthestDepth =
                D(nonEmptyNeighborT[pp], w) < D(nonEmptyNeighborB[pp], w)
                    ? D(nonEmptyNeighborT[pp], w)
                    : D(nonEmptyNeighborB[pp], w);

            if (D(nonEmptyNeighborT[pp], w) - farthestDepth <=
                DepthBlendingThreshold)
              useT = true;
            if (D(nonEmptyNeighborB[pp], w) - farthestDepth <=
                DepthBlendingThreshold)
              useB = true;
          } else {
            useT = true;
          }
        } else {
          if (nonEmptyNeighborB[pp] != -1) {
            useB = true;
          } else {
            continue;
          }
        }

        if (useT) {
          if (useB) {
            float weightT = 1.0 / abs(h - nonEmptyNeighborT[pp]);
            float weightB = 1.0 / abs(h - nonEmptyNeighborB[pp]);
            float sumWeights = weightT + weightB;

            weightT /= sumWeights;
            weightB /= sumWeights;

            Y(h, w) = Y(nonEmptyNeighborT[pp], w) * weightT;
            U(h, w) = U(nonEmptyNeighborT[pp], w) * weightT;
            V(h, w) = V(nonEmptyNeighborT[pp], w) * weightT;
            D(h, w) = D(nonEmptyNeighborT[pp], w) * weightT;

            Y(h, w) += (Y(nonEmptyNeighborB[pp], w) * weightB);
            U(h, w) += (U(nonEmptyNeighborB[pp], w) * weightB);
            V(h, w) += (V(nonEmptyNeighborB[pp], w) * weightB);
            D(h, w) += (D(nonEmptyNeighborB[pp], w) * weightB);
          } else {
            Y(h, w) = Y(nonEmptyNeighborT[pp], w);
            U(h, w) = U(nonEmptyNeighborT[pp], w);
            V(h, w) = V(nonEmptyNeighborT[pp], w);
            D(h, w) = D(nonEmptyNeighborT[pp], w);
          }
        } else {
          if (useB) {
            Y(h, w) = Y(nonEmptyNeighborB[pp], w);
            U(h, w) = U(nonEmptyNeighborB[pp], w);
            V(h, w) = V(nonEmptyNeighborB[pp], w);
            D(h, w) = D(nonEmptyNeighborB[pp], w);
          } else {
            continue;
          }
        }

      } // w
    }   // h

    delete nonEmptyNeighborL;
    delete nonEmptyNeighborR;
    delete nonEmptyNeighborT;
    delete nonEmptyNeighborB;
  }

  return;
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
