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

#ifndef _TMIV_COMMON_FRAME_H_
#error "Include the .h instead of the .hpp."
#endif

#include <stdexcept>

namespace TMIV::Common {
namespace detail {
template <> struct PixelFormatHelper<YUV400P8> {
  static constexpr int nb_plane = 1;
  static constexpr auto bitDepth = 8U;
  using base_type = std::uint8_t;
  static constexpr auto getMemorySize(int W, int H) -> int { return (W * H); }
  static constexpr auto getDiskSize(int W, int H) -> int { return (W * H) * 3 / 2; }
  static constexpr auto getPlaneWidth(int /*unused*/, int W) -> int { return W; }
  static constexpr auto getPlaneHeight(int /*unused*/, int H) -> int { return H; }
  static constexpr auto neutralColor() -> std::uint8_t { return 0x80; }
};

template <> struct PixelFormatHelper<YUV400P10> {
  static constexpr int nb_plane = 1;
  static constexpr auto bitDepth = 10U;
  using base_type = std::uint16_t;
  static constexpr auto getMemorySize(int W, int H) -> int { return 2 * (W * H); }
  static constexpr auto getDiskSize(int W, int H) -> int { return 3 * (W * H); }
  static constexpr auto getPlaneWidth(int /*unused*/, int W) -> int { return W; }
  static constexpr auto getPlaneHeight(int /*unused*/, int H) -> int { return H; }
  static constexpr auto neutralColor() -> std::uint16_t { return 0x200; }
};

template <> struct PixelFormatHelper<YUV400P16> {
  static constexpr int nb_plane = 1;
  static constexpr auto bitDepth = 16U;
  using base_type = std::uint16_t;
  static constexpr auto getMemorySize(int W, int H) -> int { return 2 * (W * H); }
  static constexpr auto getDiskSize(int W, int H) -> int { return 3 * (W * H); }
  static constexpr auto getPlaneWidth(int /*unused*/, int W) -> int { return W; }
  static constexpr auto getPlaneHeight(int /*unused*/, int H) -> int { return H; }
  static constexpr auto neutralColor() -> std::uint16_t { return 0x8000; }
};

template <> struct PixelFormatHelper<YUV420P8> {
  static constexpr int nb_plane = 3;
  static constexpr auto bitDepth = 8U;
  using base_type = std::uint8_t;
  static constexpr auto getMemorySize(int W, int H) -> int { return 3 * (W * H) / 2; }
  static constexpr auto getDiskSize(int W, int H) -> int { return 3 * (W * H) / 2; }
  static constexpr auto getPlaneWidth(int id, int W) -> int { return (id == 0) ? W : (W / 2); }
  static constexpr auto getPlaneHeight(int id, int H) -> int { return (id == 0) ? H : (H / 2); }
  static constexpr auto neutralColor() -> std::uint8_t { return 0x80; }
};

template <> struct PixelFormatHelper<YUV420P10> {
  static constexpr int nb_plane = 3;
  static constexpr auto bitDepth = 10U;
  using base_type = std::uint16_t;
  static constexpr auto getMemorySize(int W, int H) -> int { return 3 * (W * H); }
  static constexpr auto getDiskSize(int W, int H) -> int { return 3 * (W * H); }
  static constexpr auto getPlaneWidth(int id, int W) -> int { return (id == 0) ? W : (W / 2); }
  static constexpr auto getPlaneHeight(int id, int H) -> int { return (id == 0) ? H : (H / 2); }
  static constexpr auto neutralColor() -> std::uint16_t { return 0x200; }
};

template <> struct PixelFormatHelper<YUV420P16> {
  static constexpr int nb_plane = 3;
  static constexpr auto bitDepth = 16U;
  using base_type = std::uint16_t;
  static constexpr auto getMemorySize(int W, int H) -> int { return 3 * (W * H); }
  static constexpr auto getDiskSize(int W, int H) -> int { return 3 * (W * H); }
  static constexpr auto getPlaneWidth(int id, int W) -> int { return (id == 0) ? W : (W / 2); }
  static constexpr auto getPlaneHeight(int id, int H) -> int { return (id == 0) ? H : (H / 2); }
  static constexpr auto neutralColor() -> std::uint16_t { return 0x8000; }
};

template <> struct PixelFormatHelper<YUV444P8> {
  static constexpr int nb_plane = 3;
  static constexpr auto bitDepth = 8U;
  using base_type = std::uint8_t;
  static constexpr auto getMemorySize(int W, int H) -> int { return 3 * (W * H); }
  static constexpr auto getDiskSize(int W, int H) -> int { return 3 * (W * H); }
  static constexpr auto getPlaneWidth(int /*id*/, int W) -> int { return W; }
  static constexpr auto getPlaneHeight(int /*id*/, int H) -> int { return H; }
  static constexpr auto neutralColor() -> std::uint8_t { return 0x80; }
};

template <> struct PixelFormatHelper<YUV444P10> {
  static constexpr int nb_plane = 3;
  static constexpr auto bitDepth = 10U;
  using base_type = std::uint16_t;
  static constexpr auto getMemorySize(int W, int H) -> int { return 6 * (W * H); }
  static constexpr auto getDiskSize(int W, int H) -> int { return 6 * (W * H); }
  static constexpr auto getPlaneWidth(int /*id*/, int W) -> int { return W; }
  static constexpr auto getPlaneHeight(int /*id*/, int H) -> int { return H; }
  static constexpr auto neutralColor() -> std::uint16_t { return 0x200; }
};

template <> struct PixelFormatHelper<YUV444P16> {
  static constexpr int nb_plane = 3;
  static constexpr auto bitDepth = 16U;
  using base_type = std::uint16_t;
  static constexpr auto getMemorySize(int W, int H) -> int { return 6 * (W * H); }
  static constexpr auto getDiskSize(int W, int H) -> int { return 6 * (W * H); }
  static constexpr auto getPlaneWidth(int /*id*/, int W) -> int { return W; }
  static constexpr auto getPlaneHeight(int /*id*/, int H) -> int { return H; }
  static constexpr auto neutralColor() -> std::uint16_t { return 0x8000; }
};
} // namespace detail

template <class FORMAT> void Frame<FORMAT>::resize(int w, int h) {
  m_width = w;
  m_height = h;

  for (int planeId = 0; planeId < nb_plane; planeId++) {
    m_planes[planeId].resize(detail::PixelFormatHelper<FORMAT>::getPlaneHeight(planeId, h),
                             detail::PixelFormatHelper<FORMAT>::getPlaneWidth(planeId, w));
  }
}

template <class FORMAT> void Frame<FORMAT>::read(std::istream &is, bool vFlip) {
  for (auto &plane : m_planes) {
    int w = static_cast<int>(plane.width()), h = static_cast<int>(plane.height());
    base_type *ptr =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        vFlip ? (plane.data() + plane.size() - plane.width()) : plane.data();
    int lineSize = w * sizeof(base_type);

    for (int j = 0; j < h; j++) {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      is.read(reinterpret_cast<char *>(ptr), lineSize);
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      ptr = ptr + (vFlip ? -w : w);
    }
  }
}

template <class FORMAT> void Frame<FORMAT>::dump(std::ostream &os, bool vFlip) const {
  for (const auto &plane : m_planes) {
    int w = static_cast<int>(plane.width()), h = static_cast<int>(plane.height());
    const base_type *ptr =
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        vFlip ? (plane.data() + plane.size() - plane.width()) : plane.data();
    int lineSize = w * sizeof(base_type);

    for (int j = 0; j < h; j++) {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      os.write(reinterpret_cast<const char *>(ptr), lineSize);
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      ptr = ptr + (vFlip ? -w : w);
    }
  }
}

template <typename FORMAT> void Frame<FORMAT>::fillZero() {
  using base_type = typename detail::PixelFormatHelper<FORMAT>::base_type;
  for (int k = 0; k < getNumberOfPlanes(); ++k) {
    std::fill(std::begin(getPlane(k)), std::end(getPlane(k)), base_type{0});
  }
}

template <typename FORMAT> void Frame<FORMAT>::fillNeutral() {
  for (int k = 0; k < getNumberOfPlanes(); ++k) {
    std::fill(std::begin(getPlane(k)), std::end(getPlane(k)), neutralColor());
  }
}

template <typename FORMAT> void Frame<FORMAT>::fillOne() {
  for (int k = 0; k < getNumberOfPlanes(); ++k) {
    std::fill(std::begin(getPlane(k)), std::end(getPlane(k)), 1);
  }
}

template <typename FORMAT>
template <typename OTHER_FORMAT, typename>
void Frame<FORMAT>::fillInvalidWithNeutral(const Frame<OTHER_FORMAT> &depth) {
  assert(depth.getSize() == getSize());

  for (int i = 0; i < getHeight(); ++i) {
    for (int j = 0; j < getWidth(); ++j) {
      if (depth.getPlane(0)(i, j) == 0) {
        for (int k = 0; k < getNumberOfPlanes(); ++k) {
          getPlane(k)(i, j) = neutralColor();
        }
      }
    }
  }
}

template <typename FORMAT> auto AnyFrame::as() const -> Frame<FORMAT> {
  auto outputFrame = Frame<FORMAT>{static_cast<int>(planes.front().width()),
                                   static_cast<int>(planes.front().height())};
  auto &outputPlanes = outputFrame.getPlanes();
  auto maxOutputValue = (uint64_t{1} << outputFrame.getBitDepth()) - 1;

  for (size_t k = 0; k < outputPlanes.size(); ++k) {
    if (planes[k].empty()) {
      // Fill neutral when a plane is missing
      std::fill(std::begin(outputPlanes[k]), std::end(outputPlanes[k]), outputFrame.neutralColor());
    } else {
      const auto maxInputValue = (uint64_t{1} << bitdepth[k]) - 1;

      if (planes[k].size() == outputPlanes[k].size() && maxInputValue == maxOutputValue) {
        // Plane with same format: direct copy (optimization)
        std::copy(std::cbegin(planes[k]), std::cend(planes[k]), std::begin(outputPlanes[k]));
      } else {
        // Plane with different format: spatial and range scaling
        for (size_t i = 0; i < outputPlanes[k].height(); ++i) {
          const size_t n = i * planes[k].height() / outputPlanes[k].height();
          for (size_t j = 0; j < outputPlanes[k].width(); ++j) {
            const size_t m = j * planes[k].width() / outputPlanes[k].width();
            assert(planes[k](n, m) <= maxInputValue);
            using base_type = typename Frame<FORMAT>::base_type;
            outputPlanes[k](i, j) = static_cast<base_type>(
                (planes[k](n, m) * maxOutputValue + maxInputValue / 2) / maxInputValue);
            assert(outputPlanes[k](i, j) <= maxOutputValue);
          }
        }
      }
    }
  }

  return outputFrame;
}
} // namespace TMIV::Common
