/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#ifndef TMIV_MPIPCS_FRAME_H
#define TMIV_MPIPCS_FRAME_H

#include <TMIV/Common/Frame.h>

#include <memory>

namespace TMIV::MpiPcs {
struct TextureTransparency8Frame {
  TextureTransparency8Frame(Common::Frame<> texture_, Common::Frame<uint8_t> transparency_)
      : texture{std::move(texture_)}, transparency{std::move(transparency_)} {}
  Common::Frame<> texture{};
  Common::Frame<uint8_t> transparency{};
};

struct Attribute {
  using Count = uint16_t;

  using TextureValue = std::array<uint16_t, 3>;
  using GeometryValue = uint16_t;
  using TransparencyValue = uint8_t;

  static constexpr auto attributeSize = 9;
  static_assert(attributeSize ==
                sizeof(TextureValue) + sizeof(GeometryValue) + sizeof(TransparencyValue));

  using Buffer = std::array<char, attributeSize>;
  using List = std::vector<Attribute>;

  TextureValue texture{};
  GeometryValue geometry{};
  TransparencyValue transparency{};

  auto operator==(const Attribute &other) const noexcept -> bool;
  auto operator<(const Attribute &other) const noexcept -> bool {
    return (geometry < other.geometry);
  }

  static auto fromBuffer(const Buffer &buffer) -> Attribute;
  [[nodiscard]] auto toBuffer() const -> Buffer;
};

// NOTE(533#note_33297): regarding the implementation of the small vector class below
// In a word, it is just a very tailored implementation of a small vector for
// this specific case. If you observe the class MpiPcs::Frame before this commit, you can see that
// one of its attribute (m_pixelList) is a vector of vectors. In a regular scenario, the m_pixelList
// size may be up to 4500 x 2500 and the MpiEncoder may bufferized up to 32 frames (for a common
// intraperiod size). As far as I know, a regular std::vector is made of 3 pointers (8 bytes each on
// a 64 bits system) which means that, for 32 frames, the sum of each m_pixelList attribute weighs,
// at least, 4500 x 2500 x 32 x 3 x 8 = 8.64GB (with each vector being empty). If you now consider
// the implementation of the "new" Pixel class which is a small vector which takes into account that
// the number of non-empty layers per pixel will be quite small (very less than 65535), you may
// observe that the sum of each m_pixelList attribute will now be half lower. Indeed the Pixel is
// made of 2 uint16_t and 1 pointer. The former calculus now gives 4500 x 3500 x 32 x (2 x 2 +
// 8) = 4.32GB i.e. more than 4GB saved. Smarter implementation to save even more memory could be
// envisioned (with look-up tables), but you would lose in usability from my point of view.
class Pixel {
public:
  using value_type = Attribute;
  using size_type = uint16_t;
  // NOTE(#488): This low-level construct was chosen to reduce the memory footprint of MPI frames.
  //
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)
  using array_type = value_type[];

  using DataPtr = std::unique_ptr<array_type>;
  using iterator = DataPtr::pointer;

  Pixel() = default;
  explicit Pixel(size_type sz);
  ~Pixel() = default;
  Pixel(const Pixel &other);
  Pixel(Pixel &&other) noexcept;
  auto operator=(const Pixel &other) -> Pixel &;
  auto operator=(Pixel &&other) noexcept -> Pixel &;
  [[nodiscard]] auto capacity() const -> size_type { return m_capacity; }
  [[nodiscard]] auto size() const -> size_type { return m_size; }
  [[nodiscard]] auto empty() const -> bool { return (m_size == 0); }
  void clear() { m_size = 0; }
  void reserve(size_type sz);
  auto begin() -> iterator { return m_data.get(); }
  [[nodiscard]] auto begin() const -> iterator { return m_data.get(); }
  auto end() -> iterator { return m_data.get() + m_size; }
  [[nodiscard]] auto end() const -> iterator { return m_data.get() + m_size; }
  auto operator[](size_type k) -> value_type & { return m_data.get()[k]; }
  auto operator[](size_type k) const -> const value_type & { return m_data.get()[k]; }
  auto operator==(const Pixel &other) const noexcept -> bool;
  void push_back(const value_type &v);

private:
  size_type m_size{};
  size_type m_capacity{};
  DataPtr m_data{};
};

class Frame {
public:
  Frame() = default;
  Frame(const Common::Vec2i &size)
      : m_size{size}, m_pixelList{static_cast<size_t>(m_size.x() * m_size.y())} {}
  Frame(const Common::Vec2i &size, std::vector<Pixel> pixelList)
      : m_size{size}, m_pixelList{std::move(pixelList)} {}
  auto operator==(const Frame &other) const noexcept -> bool;
  [[nodiscard]] auto getPixelList() const -> const std::vector<Pixel> & { return m_pixelList; }
  auto operator()(int32_t i, int32_t j) const -> const Pixel & {
    ASSERT(i * m_size.x() + j < static_cast<int32_t>(m_pixelList.size()));
    return m_pixelList[i * m_size.x() + j];
  }
  void appendLayer(Attribute::GeometryValue layerId, const TextureTransparency8Frame &layer);
  [[nodiscard]] auto getLayer(Attribute::GeometryValue layerId) const -> TextureTransparency8Frame;

private:
  Common::Vec2i m_size{};
  std::vector<Pixel> m_pixelList{};
};
} // namespace TMIV::MpiPcs

#endif
