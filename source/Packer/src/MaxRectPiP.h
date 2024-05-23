/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ISO/IEC
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

#ifndef TMIV_PACKER_MAXRECTPIP_H
#define TMIV_PACKER_MAXRECTPIP_H

#include <TMIV/Packer/Cluster.h>
#include <list>

namespace TMIV::Packer {
class MaxRectPiP {
public:
  class Output {
  private:
    int32_t m_x = 0;
    int32_t m_y = 0;
    bool m_isRotated = false;

  public:
    void set(int32_t x, int32_t y, bool isRotated) {
      m_x = x;
      m_y = y;
      m_isRotated = isRotated;
    }
    [[nodiscard]] auto x() const -> int32_t { return m_x; }
    [[nodiscard]] auto y() const -> int32_t { return m_y; }
    [[nodiscard]] auto isRotated() const -> bool { return m_isRotated; }
  };
  using OccupancyMap = TMIV::Common::Mat<uint8_t>;

private:
  class Rectangle {
  private:
    int32_t m_x0 = 0;
    int32_t m_y0 = 0;
    int32_t m_x1 = 0;
    int32_t m_y1 = 0;

  public:
    Rectangle() = default;
    Rectangle(int32_t x0, int32_t y0, int32_t x1, int32_t y1)
        : m_x0(x0), m_y0(y0), m_x1(x1), m_y1(y1) {}
    [[nodiscard]] auto left() const -> int32_t { return m_x0; }
    [[nodiscard]] auto right() const -> int32_t { return m_x1; }
    [[nodiscard]] auto bottom() const -> int32_t { return m_y0; }
    [[nodiscard]] auto top() const -> int32_t { return m_y1; }
    [[nodiscard]] auto width() const -> int32_t { return (m_x1 - m_x0 + 1); }
    [[nodiscard]] auto height() const -> int32_t { return (m_y1 - m_y0 + 1); }
    [[nodiscard]] auto split(int32_t w, int32_t h) const -> std::vector<Rectangle>;
    [[nodiscard]] auto remove(const Rectangle &r) const -> std::vector<Rectangle>;
    [[nodiscard]] auto isInside(const Rectangle &r) const -> bool;
    [[nodiscard]] auto getShortSideFitScore(int32_t w, int32_t h) const -> float;
    [[nodiscard]] auto getArea() const -> int32_t { return (width() * height()); }
  };

  int32_t m_alignment = 0;
  std::list<Rectangle> m_F;
  bool m_pip = true;
  OccupancyMap m_occupancyMap;

public:
  MaxRectPiP(int32_t w, int32_t h, int32_t a, bool pip);
  auto push(const Cluster &c, const ClusteringMap &clusteringMap, Output &packerOutput) -> bool;
  auto setIsPushInFreeSpace(bool value) { m_isPushInFreeSpace = value; }
  auto getIsPushInFreeSpace() const -> bool { return m_isPushInFreeSpace; }

  auto setAreaPatchPushedInFreeSpace(int32_t area) {
    m_areaOfPatchPushedInFreeSpace = m_areaOfPatchPushedInFreeSpace + area;
  }
  auto getAreaPatchPushedInFreeSpace() const -> int32_t { return m_areaOfPatchPushedInFreeSpace; }

private:
  void updateOccupancyMap(const Cluster &c, const ClusteringMap &clusteringMap,
                          const Output &packerOutput);
  auto pushInUsedSpace(int32_t w, int32_t h, bool isBasicView, Output &packerOutput) -> bool;
  auto pushInFreeSpace(int32_t w, int32_t h, bool isBasicView, Output &packerOutput) -> bool;

  bool m_isPushInFreeSpace{};
  int32_t m_areaOfPatchPushedInFreeSpace{};
};
} // namespace TMIV::Packer

#endif
