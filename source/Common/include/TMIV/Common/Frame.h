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

#ifndef TMIV_COMMON_FRAME_H
#define TMIV_COMMON_FRAME_H

#include "Common.h"
#include "Matrix.h"
#include "Vector.h"

#include <array>
#include <cstdint>
#include <istream>
#include <ostream>
#include <variant>

namespace TMIV::Common {
enum class ColorFormat { YUV400, YUV420, YUV444 };

template <typename Element = DefaultElement> class Frame {
private:
  static_assert(std::is_unsigned_v<Element>);

  static constexpr uint32_t maxBitDepth = std::numeric_limits<Element>::digits;
  static_assert(1 <= maxBitDepth && maxBitDepth <= 32);
  uint32_t m_bitDepth{UINT32_MAX};

  using Plane = Mat<Element>;
  using Planes = std::vector<Plane>;
  Planes m_planes{};

public:
  // Constuct an empty frame of unspecified bit depth
  Frame() = default;

  // Construct a frame of given size and bit depth with all elements set to zero
  explicit Frame(Common::Vec2i frameSize, uint32_t bitDepth, ColorFormat colorFormat) {
    create(frameSize, bitDepth, colorFormat);
  }

  // Construct a luma-only frame of given size and bit depth, all zero
  static auto lumaOnly(Common::Vec2i frameSize, uint32_t bitDepth = maxBitDepth) {
    return Frame{frameSize, bitDepth, ColorFormat::YUV400};
  }

  // Construct a 4:2:0 frame of given size and bit depth, all zero
  static auto yuv420(Common::Vec2i frameSize, uint32_t bitDepth = maxBitDepth) {
    return Frame{frameSize, bitDepth, ColorFormat::YUV420};
  }

  // Construct a 4:4:4 frame of given size and bit depth, all zero
  static auto yuv444(Common::Vec2i frameSize, uint32_t bitDepth = maxBitDepth) {
    return Frame{frameSize, bitDepth, ColorFormat::YUV444};
  }

  // Create a frame of given size, bit depth and color format, all zero
  void create(Common::Vec2i frameSize, uint32_t bitDepth, ColorFormat colorFormat);

  // Create a luma-only frame of given size and bit depth, all zero
  void createY(Common::Vec2i frameSize, uint32_t bitDepth = maxBitDepth) {
    create(frameSize, bitDepth, ColorFormat::YUV400);
  }

  // Create a 4:2:0 frame of given size and bit depth, all zero
  void createYuv420(Common::Vec2i frameSize, uint32_t bitDepth = maxBitDepth) {
    create(frameSize, bitDepth, ColorFormat::YUV420);
  }

  // Create a 4:4:4 frame of given size and bit depth, all zero
  void createYuv444(Common::Vec2i frameSize, uint32_t bitDepth = maxBitDepth) {
    create(frameSize, bitDepth, ColorFormat::YUV444);
  }

  void clear() { m_planes.clear(); }

  [[nodiscard]] auto empty() const noexcept { return m_planes.empty(); }

  [[nodiscard]] auto getPlanes() noexcept -> auto & { return m_planes; }

  [[nodiscard]] auto getPlanes() const noexcept -> const auto & { return m_planes; }

  // Mutable access to a plane. The caller shall not resize the plane
  template <typename Integral, typename = std::enable_if_t<std::is_integral_v<Integral>>>
  [[nodiscard]] auto getPlane(Integral index) -> auto & {
    return at(m_planes, index);
  }

  template <typename Integral, typename = std::enable_if_t<std::is_integral_v<Integral>>>
  [[nodiscard]] auto getPlane(Integral index) const -> const auto & {
    return at(m_planes, index);
  }

  [[nodiscard]] auto getWidth() const noexcept {
    PRECONDITION(!empty());
    return static_cast<int32_t>(m_planes.front().size(1));
  }

  [[nodiscard]] auto getHeight() const noexcept {
    PRECONDITION(!empty());
    return static_cast<int32_t>(m_planes.front().size(0));
  }

  [[nodiscard]] auto getSize() const noexcept { return Vec2i{getWidth(), getHeight()}; }

  [[nodiscard]] auto getNumberOfPlanes() const noexcept { return m_planes.size(); }

  // Derive the color format of a non-empty plane from the plane count and sizes, assuming it is one
  // of the known color formats.
  [[nodiscard]] auto getColorFormat() const noexcept -> ColorFormat;

  // Query the bit depth of a non-empty frame
  [[nodiscard]] auto getBitDepth() const noexcept {
    PRECONDITION(m_bitDepth <= maxBitDepth);
    return m_bitDepth;
  }

  // Read the frame from a stream with the same native element (regardless of bit depth)
  void readFrom(std::istream &stream);

  // Write the frame to a stream with the same native element (regardless of bit depth)
  void writeTo(std::ostream &stream) const;

  [[nodiscard]] auto getByteCount() const noexcept;

  // Return the minimum value (assuming full range)
  [[nodiscard]] static constexpr auto minValue() noexcept { return Element{}; }

  // Return the neutral value at the current bit depth
  [[nodiscard]] auto neutralValue() const noexcept -> Element {
    return Common::medLevel<Element>(getBitDepth());
  }

  // Return the maximum value at the current bit depth (assuming full range)
  [[nodiscard]] auto maxValue() const noexcept -> Element {
    return Common::maxLevel<Element>(getBitDepth());
  }

  // Reset all samples to zero
  void fillZero() noexcept { fillValue(minValue()); }

  // Set all samples to one
  void fillOne() noexcept { fillValue(static_cast<Element>(1)); }

  // Set all samples to the neutral (halfway) value
  void fillNeutral() noexcept { fillValue(neutralValue()); }

  // Reset all samples to the maximum value
  void fillMax() noexcept { fillValue(maxValue()); }

  // Set all samples to a specific value
  void fillValue(Element value) noexcept;

  // Fill the values that are marked as invalid (0) in the mask with the specified value
  //
  //   * Only the first plane of the mask is considered.
  //   * All planes of the frame must have the same size as the mask.
  template <typename OtherElement>
  void fillInvalid(const Frame<OtherElement> &mask, Element value) noexcept;

  // Fill the values that are marked as invalid (0) in the mask with the neutral value
  //
  //   * Only the first plane of the mask is considered.
  //   * All planes of the frame must have the same size as the mask.
  template <typename OtherElement>
  void fillInvalidWithNeutral(const Frame<OtherElement> &mask) noexcept;

  // Change (or set) the bit depth without changing the data, color format or frame size
  auto setBitDepth(uint32_t value) noexcept;
};

using PatchIdx = uint16_t;
static constexpr auto unusedPatchIdx = UINT16_MAX;

struct DeepFrame {
  Frame<> texture;
  Frame<> geometry;
  Frame<> entities{};
  Frame<> occupancy{};
  Frame<> transparency{};
  Frame<> packed{};
  Frame<> patchIdx{};
};

struct RendererFrame {
  Common::Frame<> texture;
  Common::Frame<> geometry;
};

struct DecodedFrame : public Common::Frame<> {
  DecodedFrame() = default;
  DecodedFrame(Common::Frame<> frame, bool irap_)
      : Common::Frame<>{std::move(frame)}, irap{irap_} {}

  bool irap{};
};

template <typename Element = DefaultElement> using FrameList = std::vector<Frame<Element>>;
using DeepFrameList = std::vector<DeepFrame>;

// Expand a YUV 4:4:4 texture to packed 4:4:4 32-bit float texture with linear transfer and nearest
// interpolation for chroma
auto expandTexture(const Frame<> &inYuv) -> Mat<Vec3f>;

// Expand a YUV 4:2:0 texture to 32-bit float luma map with linear transfer
auto expandLuma(const Frame<> &inYuv) -> Mat<float>;

// Quantize a packed 4:4:4 32-bit float texture as YUV 4:4:4 texture with linear transfer and area
// interpolation for chroma
auto quantizeTexture(const Mat<Vec3f> &in, uint32_t bitDepth) -> Frame<>;

template <typename Element> [[nodiscard]] auto yuv400(const Frame<Element> &frame);
template <typename Element> [[nodiscard]] auto yuv420(const Frame<Element> &frame);
template <typename Element> [[nodiscard]] auto yuv444(const Frame<Element> &frame);

template <typename OtherElement, typename Element>
[[nodiscard]] auto elementCast(const Frame<Element> &frame);
} // namespace TMIV::Common

#include "Frame.hpp"

#endif
