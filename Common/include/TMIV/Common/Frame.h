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

#ifndef _TMIV_COMMON_FRAME_H_
#define _TMIV_COMMON_FRAME_H_

#include <TMIV/Common/Matrix.h>
#include <TMIV/Common/Vector.h>

#include <cassert>
#include <cstdint>
#include <istream>
#include <ostream>

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
  Frame &operator=(const Frame &) = default;
  Frame &operator=(Frame &&that) noexcept {
    m_width = that.m_width;
    m_height = that.m_height;
    m_planes = std::move(that.m_planes);

    that.m_width = 0;
    that.m_height = 0;

    return *this;
  }
  ~Frame() = default;

  void resize(int w, int h);

  std::array<plane_type, nb_plane> &getPlanes() { return m_planes; }
  const std::array<plane_type, nb_plane> &getPlanes() const { return m_planes; }

  const plane_type &getPlane(int id) const { return m_planes[id]; }
  plane_type &getPlane(int id) { return m_planes[id]; }
  int getWidth() const { return m_width; }
  int getHeight() const { return m_height; }
  Vec2i getSize() const { return Vec2i{m_width, m_height}; }
  int getMemorySize() const {
    return detail::PixelFormatHelper<FORMAT>::getMemorySize(m_width, m_height);
  }
  int getDiskSize() const {
    return detail::PixelFormatHelper<FORMAT>::getDiskSize(m_width, m_height);
  }
  static constexpr int getNumberOfPlanes() { return nb_plane; }
  static constexpr int getBitDepth() { return bitDepth; }

  void read(std::istream &is, bool vFlip = false);
  void dump(std::ostream &os, bool vFlip = false) const;

  // Reset all samples to zero
  //
  // Note that samples are already set to zero on construction
  void fillZero();

  // Set all samples to the neutral color
  void fillNeutral();

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
} // namespace TMIV::Common

#include "Frame.hpp"

namespace TMIV::Common {
using TextureFrame = Frame<YUV420P10>;
using Texture444Frame = Frame<YUV444P10>; // The renderer uses 4:4:4 internally
using Depth10Frame = Frame<YUV400P10>;    // Decoder side
using Depth16Frame = Frame<YUV400P16>;    // Encoder side
using Mask = Frame<YUV400P8>;
using PatchIdMap = Frame<YUV400P16>;
using EntityMap = Frame<YUV400P16>;

// TODO(BK): Rename struct and data members after TMIV-4.0alpha1 milestone
template <typename FORMAT> struct TextureDepthFrame {

  using first_type = TextureFrame;
  using second_type = Frame<FORMAT>;

  TextureFrame first;
  Frame<FORMAT> second;
  EntityMap entities{};

  TextureDepthFrame() = default;
  TextureDepthFrame(TextureFrame texture_, Frame<FORMAT> depth_)
      : first{std::move(texture_)}, second{std::move(depth_)} {}
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
using PatchIdMapList = std::vector<PatchIdMap>;

const auto unusedPatchId = std::uint16_t(65535);
} // namespace TMIV::Common

#endif
