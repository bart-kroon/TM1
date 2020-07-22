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
#define _TMIV_COMMON_FRAME_H_

#include <TMIV/Common/Matrix.h>
#include <TMIV/Common/Vector.h>

#include <cassert>
#include <cstdint>
#include <istream>
#include <ostream>
#include <variant>

namespace TMIV::Common {
class YUV400P8 {};
class YUV400P10 {};
class YUV400P16 {};
class YUV420P8 {};
class YUV420P10 {};
class YUV420P16 {};
class YUV444P8 {};
class YUV444P10 {};
class YUV444P16 {};

namespace detail {
template <class FORMAT> struct PixelFormatHelper {};
} // namespace detail

template <class FORMAT> constexpr auto neutralColor() {
  return detail::PixelFormatHelper<FORMAT>::neutralColor();
}

template <class FORMAT> class Frame {
public:
  using base_type = typename detail::PixelFormatHelper<FORMAT>::base_type;
  using plane_type = heap::Matrix<base_type>;

protected:
  static constexpr int nb_plane = detail::PixelFormatHelper<FORMAT>::nb_plane;
  static constexpr int bitDepth = detail::PixelFormatHelper<FORMAT>::bitDepth;

protected:
  int m_width = 0, m_height = 0;
  std::array<plane_type, nb_plane> m_planes;

public:
  explicit Frame(int w = 0, int h = 0) { resize(w, h); }
  Frame(const Frame &) = default;
  Frame(Frame &&that) noexcept {
    m_width = that.m_width;
    m_height = that.m_height;
    m_planes = std::move(that.m_planes);

    that.m_width = 0;
    that.m_height = 0;
  }
  auto operator=(const Frame &) -> Frame & = default;
  auto operator=(Frame &&that) noexcept -> Frame & {
    m_width = that.m_width;
    m_height = that.m_height;
    m_planes = std::move(that.m_planes);

    that.m_width = 0;
    that.m_height = 0;

    return *this;
  }
  ~Frame() = default;

  [[nodiscard]] auto empty() const noexcept -> bool { return getWidth() == 0 && getHeight() == 0; }

  void resize(int w, int h);

  auto getPlanes() -> std::array<plane_type, nb_plane> & { return m_planes; }
  [[nodiscard]] auto getPlanes() const -> const std::array<plane_type, nb_plane> & {
    return m_planes;
  }

  [[nodiscard]] auto getPlane(int id) const -> const plane_type & { return m_planes[id]; }
  auto getPlane(int id) -> plane_type & { return m_planes[id]; }
  [[nodiscard]] auto getWidth() const -> int { return m_width; }
  [[nodiscard]] auto getHeight() const -> int { return m_height; }
  [[nodiscard]] auto getSize() const -> Vec2i { return Vec2i{m_width, m_height}; }
  [[nodiscard]] auto getMemorySize() const -> int {
    return detail::PixelFormatHelper<FORMAT>::getMemorySize(m_width, m_height);
  }
  [[nodiscard]] auto getDiskSize() const -> int {
    return detail::PixelFormatHelper<FORMAT>::getDiskSize(m_width, m_height);
  }
  static constexpr auto getNumberOfPlanes() -> int { return nb_plane; }
  static constexpr auto getBitDepth() -> int { return bitDepth; }

  void read(std::istream &is, bool vFlip = false);
  void dump(std::ostream &os, bool vFlip = false) const;

  // Reset all samples to zero
  //
  // Note that samples are already set to zero on construction
  void fillZero();

  // Set all samples to the neutral color
  void fillNeutral();

  // Set all samples to one
  void fillOne();

  // Set invalid samples to the neutral color
  template <typename OTHER_FORMAT, typename = std::enable_if<std::is_same_v<FORMAT, YUV444P10>>>
  void filIInvalidWithNeutral(const Frame<OTHER_FORMAT> &depth);

  static constexpr auto neutralColor() { return detail::PixelFormatHelper<FORMAT>::neutralColor(); }
};

auto yuv420p(const Frame<YUV444P8> &frame) -> Frame<YUV420P8>;
auto yuv420p(const Frame<YUV444P10> &frame) -> Frame<YUV420P10>;
auto yuv420p(const Frame<YUV444P16> &frame) -> Frame<YUV420P16>;

auto yuv444p(const Frame<YUV420P8> &frame) -> Frame<YUV444P8>;
auto yuv444p(const Frame<YUV420P10> &frame) -> Frame<YUV444P10>;
auto yuv444p(const Frame<YUV420P16> &frame) -> Frame<YUV444P16>;

// A type that can carry a large variation of possible frame types
//
// TODO(BK): Consider using AnyFrame for IO library
struct AnyFrame {
  AnyFrame() = default;

  // Convert from any specific format
  template <typename FORMAT> explicit AnyFrame(const Frame<FORMAT> &frame);

  // Convert to any specific format
  template <typename FORMAT> auto as() const -> Frame<FORMAT>;

  static constexpr int maxPlanes = 4;
  std::array<Mat<uint32_t>, maxPlanes> planes{};
  std::array<uint8_t, maxPlanes> bitdepth{};
};
} // namespace TMIV::Common

#include "Frame.hpp"

namespace TMIV::Common {
using TextureFrame = Frame<YUV420P10>;
using Texture444Frame = Frame<YUV444P10>; // The renderer uses 4:4:4 internally
using Depth10Frame = Frame<YUV400P10>;    // Decoder side
using Depth16Frame = Frame<YUV400P16>;    // Encoder side
using Occupancy10Frame = Frame<YUV400P10>;
using Mask = Frame<YUV400P8>;
using BlockToPatchMap = Frame<YUV400P16>;
const auto unusedPatchId = std::uint16_t(65535);
using EntityMap = Frame<YUV400P16>;

template <typename FORMAT> struct TextureDepthFrame {
  using first_type = TextureFrame;
  using second_type = Frame<FORMAT>;

  TextureFrame texture;
  Frame<FORMAT> depth;
  EntityMap entities{};
  Occupancy10Frame occupancy{};

  TextureDepthFrame() = default;
  TextureDepthFrame(TextureFrame texture_, Frame<FORMAT> depth_)
      : texture{std::move(texture_)}, depth{std::move(depth_)} {}
  TextureDepthFrame(TextureFrame texture_, Frame<FORMAT> depth_, Occupancy10Frame occupancy_)
      : texture{std::move(texture_)}, depth{std::move(depth_)}, occupancy{std::move(occupancy_)} {}
};
using TextureDepth10Frame = TextureDepthFrame<YUV400P10>;
using TextureDepth16Frame = TextureDepthFrame<YUV400P16>;
using Texture444Depth10Frame = std::pair<Texture444Frame, Depth10Frame>;
using Texture444Depth16Frame = std::pair<Texture444Frame, Depth16Frame>;

using EntityMapList = std::vector<EntityMap>;
template <typename FORMAT> using MVDFrame = std::vector<TextureDepthFrame<FORMAT>>;
using MVD10Frame = MVDFrame<YUV400P10>;
using MVD16Frame = MVDFrame<YUV400P16>;
using MaskList = std::vector<Mask>;

// Expand a YUV 4:4:4 10-bit texture to packed 4:4:4 32-bit float texture with
// linear transfer and nearest interpolation for chroma
auto expandTexture(const Common::Frame<Common::YUV444P10> &inYuv) -> Common::Mat<Common::Vec3f>;

// Expand a YUV 4:2:0 10-bit texture to 32-bit float luma map with linear transfer
auto expandLuma(const Common::Frame<Common::YUV420P10> &inYuv) -> Common::Mat<float>;

// Quantize a packed 4:4:4 32-bit float texture as YUV 4:4:4 10-bit texture with
// linear transfer and area interpolation for chroma
auto quantizeTexture(const Common::Mat<Common::Vec3f> &in) -> Common::Frame<Common::YUV444P10>;
} // namespace TMIV::Common

#endif
