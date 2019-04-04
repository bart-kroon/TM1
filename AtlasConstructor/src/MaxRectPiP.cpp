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

#include "MaxRectPiP.h"
#include <TMIV/Common/LinAlg.h>

namespace TMIV::AtlasConstructor {

////////////////////////////////////////////////////////////////////////////////
std::vector<MaxRectPiP::Rectangle>
MaxRectPiP::Rectangle::remove(const Rectangle &r) const {
  std::vector<Rectangle> out;

  if (!((r.m_x1 <= m_x0) || (m_x1 <= r.m_x0) || (r.m_y1 <= m_y0) ||
        (m_y1 <= r.m_y0))) {
    // Left part
    if (m_x0 < r.m_x0)
      out.push_back(Rectangle(m_x0, m_y0, r.m_x0 - 1, m_y1));

    // Right part
    if (r.m_x1 < m_x1)
      out.push_back(Rectangle(r.m_x1 + 1, m_y0, m_x1, m_y1));

    // Bottom part
    if (m_y0 < r.m_y0)
      out.push_back(Rectangle(m_x0, m_y0, m_x1, r.m_y0 - 1));

    // Top part
    if (r.m_y1 < m_y1)
      out.push_back(Rectangle(m_x0, r.m_y1 + 1, m_x1, m_y1));
  }

  return out;
}

bool MaxRectPiP::Rectangle::isInside(const Rectangle &r) const {
  return ((r.m_x0 <= m_x0) && (m_x0 <= r.m_x1) && (r.m_x0 <= m_x1) &&
          (m_x1 <= r.m_x1) && (r.m_y0 <= m_y0) && (m_y0 <= r.m_y1) &&
          (r.m_y0 <= m_y1) && (m_y1 <= r.m_y1));
}

float MaxRectPiP::Rectangle::getShortSideFitScore(int w, int h) const {
  int dw = width() - w, dh = height() - h;

  if ((0 <= dw) && (0 <= dh))
    return (float) (std::min)(dw, dh);
  else
    return std::numeric_limits<float>::max();
}

//////////////////////////////////////////////////////////////
MaxRectPiP::MaxRectPiP(int w, int h, int a, bool pip)
    : m_width(w), m_height(h), m_alignment(a), m_pip(pip) {
  // Maps
  unsigned int wa = w / a, ha = h / a;

  m_occupancyMap.resize({ha, wa});
  std::fill(m_occupancyMap.begin(), m_occupancyMap.end(), 0);

  // Push full rectangle
  m_F.push_back(Rectangle(0, 0, w - 1, h - 1));
}

bool MaxRectPiP::push(const Cluster &c, const ClusteringMap &clusteringMap,
                      Output &packerOutput) {
  int w = Common::align(c.width(), m_alignment),
      h = Common::align(c.height(), m_alignment);

  if ((m_pip && pushInUsedSpace(w, h, packerOutput)) ||
      pushInFreeSpace(w, h, packerOutput)) {
    // Update occupancy map
    updateOccupancyMap(c, clusteringMap, packerOutput);

    return true;
  } else
    return false;
}

void MaxRectPiP::updateOccupancyMap(const Cluster &c,
                                    const ClusteringMap &clusteringMap,
                                    const MaxRectPiP::Output &packerOutput) {
  using namespace TMIV::Common;

  Vec2i mappingPosition = {c.jmin(), c.imin()};
  Vec2i mappingSize = {c.width(), c.height()};
  Vec2i packingPosition = {packerOutput.x(), packerOutput.y()};
  Vec2i packingSize = packerOutput.isRotated()
                          ? Vec2i({mappingSize[1], mappingSize[0]})
                          : mappingSize;

  Mat3x3i Q2P;

  if (packerOutput.isRotated()) {
    Q2P = {
        0, -1, (mappingSize[0] - 1) + (packingPosition[1] + mappingPosition[0]),
        1, 0,  mappingPosition[1] - packingPosition[0],
        0, 0,  1};
  } else {
    Q2P = {1, 0, mappingPosition[0] - packingPosition[0],
           0, 1, mappingPosition[1] - packingPosition[1],
           0, 0, 1};
  }

  int xMin = packingPosition[0], xMax = packingPosition[0] + packingSize[0] - 1;
  int yMin = packingPosition[1], yMax = packingPosition[1] + packingSize[1] - 1;

  int XMin = xMin / m_alignment, XMax = xMax / m_alignment;
  int YMin = yMin / m_alignment, YMax = yMax / m_alignment;

  for (int Y = YMin; Y <= YMax; Y++) {
    int y0 = std::max(yMin, Y * m_alignment),
        y1 = std::min(yMax, y0 + m_alignment);

    for (int X = XMin; X <= XMax; X++) {
      int x0 = std::max(xMin, X * m_alignment),
          x1 = std::min(xMax, x0 + m_alignment);

      bool available = true;

      for (int y = y0; y < y1; y++) {
        for (int x = x0; x < x1; x1++) {
          Vec3i p = Q2P * Vec3i({x, y, 1});

          if (clusteringMap(p.y(), p.x()) ==
              c.getClusterId()) // (inRange(p.x(), 0, wMinusOne) &&
                                // inRange(p.y(), 0, hMinusOne)) &&
          {
            available = false;
            break;
          }
        }

        if (!available) {
          m_occupancyMap(Y, X) = 255;
          break;
        }
      }

      if (available)
        m_occupancyMap(Y, X) = 128;
    }
  }
}

bool MaxRectPiP::pushInUsedSpace(int w, int h,
                                 MaxRectPiP::Output &packerOutput) {


  int W = w / m_alignment, H = h / m_alignment;

  auto isGoodCandidate = [this](int xmin, int xmax, int ymin,
                                int ymax) -> bool {
    if ((xmax < (int)m_occupancyMap.width()) &&
        (ymax < (int)m_occupancyMap.height())) {
      for (int y = ymin; y <= ymax; y++) {
        for (int x = xmin; x <= xmax; x++) {
          if (m_occupancyMap(y, x) != 128)
            return false;
        }
      }

      return true;
    } else
      return false;
  };

  for (auto Y = 0u; Y < m_occupancyMap.height(); Y++) {
    for (auto X = 0u; X < m_occupancyMap.width(); X++) {
      // Without Rotation
      if (isGoodCandidate(X, X + W - 1, Y, Y + H - 1)) {
        packerOutput.set(X * m_alignment, Y * m_alignment, false);
        return true;
      }

      // With Rotation
      if (isGoodCandidate(X, X + H - 1, Y, Y + W - 1)) {
        packerOutput.set(X * m_alignment, Y * m_alignment, true);
        return true;
      }
    }
  }

  return false;
}

bool MaxRectPiP::pushInFreeSpace(int w, int h,
                                 MaxRectPiP::Output &packerOutput) {
  // Select best free rectangles that fit current patch (BSSF criterion)
  std::list<Rectangle>::const_iterator best_iter = m_F.cend();
  float best_score = std::numeric_limits<float>::max();
  bool best_rotation = false;

  for (auto iter = m_F.cbegin(); iter != m_F.cend(); iter++) {
    const Rectangle &r = *iter;
    float score_regular = r.getShortSideFitScore(w, h);
    float score_rotated = r.getShortSideFitScore(h, w);

    if ((score_regular <= score_rotated) && (score_regular < best_score)) {
      best_iter = iter;
      best_score = score_regular;
      best_rotation = false;
    } else if (score_rotated < best_score) {
      best_iter = iter;
      best_score = score_rotated;
      best_rotation = true;
    }
  }

  if (best_iter == m_F.cend())
    return false;

  // Update free rectangles
  Rectangle B(best_iter->left(), best_iter->bottom(),
              best_iter->left() + ((best_rotation ? h : w) - 1),
              best_iter->bottom() + ((best_rotation ? w : h) - 1));

  // Split current free rectangle
  std::vector<Rectangle> splitted = best_iter->split(B.width(), B.height());
  std::move(splitted.begin(), splitted.end(), std::back_inserter(m_F));

  m_F.erase(best_iter);

  // Intersection with existing free rectangles
  std::vector<std::list<Rectangle>::const_iterator> intersected;
  std::list<Rectangle>::const_iterator last = m_F.cend();

  for (auto iter = m_F.cbegin(); iter != last; iter++) {
    std::vector<Rectangle> intersecting = iter->remove(B);

    if (!intersecting.empty()) {
      intersected.push_back(iter);
      std::move(intersecting.begin(), intersecting.end(),
                std::back_inserter(m_F));
    }
  }

  for (const auto &iter : intersected)
    m_F.erase(iter);

  // Degenerated free rectangles
  std::vector<std::list<Rectangle>::const_iterator> degenerated;

  for (auto iter1 = m_F.cbegin(); iter1 != m_F.cend(); iter1++) {
    for (auto iter2 = m_F.cbegin(); iter2 != m_F.cend(); iter2++) {
      if ((iter1 != iter2) && (iter1->isInside(*iter2))) {
        degenerated.push_back(iter1);
        break;
      }
    }
  }

  for (const auto &iter : degenerated)
    m_F.erase(iter);

  // Update output
  packerOutput.set(B.left(), B.bottom(), best_rotation);

  return true;
}

} // namespace TMIV::AtlasConstructor
