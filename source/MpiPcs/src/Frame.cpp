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

#include <TMIV/MpiPcs/Frame.h>

#include <TMIV/Common/Bytestream.h>
#include <TMIV/Common/Common.h>
#include <TMIV/Common/Thread.h>

#include <sstream>

namespace TMIV::MpiPcs {
using Common::getUint16;
using Common::getUint8;
using Common::putUint16;
using Common::putUint8;
using Common::swapEndianness;

auto Attribute::operator==(const Attribute &other) const noexcept -> bool {
  return texture == other.texture && geometry == other.geometry &&
         transparency == other.transparency;
}

auto Attribute::fromBuffer(const Buffer &buffer) -> Attribute {
  const auto blob = std::string{buffer.cbegin(), buffer.cend()}; // likely SSO
  std::istringstream stream{blob};

  Attribute a;
  a.texture[0] = swapEndianness(getUint16(stream));
  a.texture[1] = swapEndianness(getUint16(stream));
  a.texture[2] = swapEndianness(getUint16(stream));
  a.geometry = swapEndianness(getUint16(stream));
  a.transparency = getUint8(stream);

  return a;
}

auto Attribute::toBuffer() const -> Buffer {
  auto buffer = Buffer{};

  std::ostringstream stream;
  putUint16(stream, swapEndianness(texture[0]));
  putUint16(stream, swapEndianness(texture[1]));
  putUint16(stream, swapEndianness(texture[2]));
  putUint16(stream, swapEndianness(geometry));
  putUint8(stream, transparency);

  const auto blob = stream.str(); // likely SSO
  static_assert(buffer.size() == attributeSize);
  ASSERT(blob.size() == buffer.size());

  std::copy(blob.cbegin(), blob.cend(), buffer.begin());
  return buffer;
}

Pixel::Pixel(size_type sz) {
  m_size = sz;
  m_capacity = sz;
  m_data = std::make_unique<array_type>(m_capacity);
}

Pixel::Pixel(const Pixel &other) {
  m_size = other.m_size;
  m_capacity = m_size;
  m_data = std::make_unique<array_type>(m_capacity);

  std::copy(other.begin(), other.end(), begin());
}

Pixel::Pixel(Pixel &&other) noexcept {
  m_size = other.m_size;
  m_capacity = other.m_capacity;
  m_data = std::move(other.m_data);

  other.m_size = 0;
  other.m_capacity = 0;
  other.m_data = nullptr;
}

auto Pixel::operator=(const Pixel &other) -> Pixel & {
  if (m_size != other.m_size) {
    m_size = other.m_size;
    m_capacity = m_size;
    m_data = std::make_unique<array_type>(m_capacity);
  }

  std::copy(other.begin(), other.end(), begin());

  return *this;
}

auto Pixel::operator=(Pixel &&other) noexcept -> Pixel & {
  m_size = other.m_size;
  m_capacity = other.m_capacity;
  m_data = std::move(other.m_data);

  other.m_size = 0;
  other.m_capacity = 0;
  other.m_data = nullptr;

  return *this;
}

void Pixel::reserve(size_type sz) {
  if (m_capacity < sz) {
    m_capacity = sz;

    auto data = std::make_unique<array_type>(m_capacity);
    std::copy(begin(), end(), data.get());

    m_data = std::move(data);
  }
}

auto Pixel::operator==(const Pixel &other) const noexcept -> bool {
  return (size() == other.size()) && std::equal(begin(), end(), other.begin());
}

void Pixel::push_back(const value_type &v) {
  if (m_size < m_capacity) {
    m_data.get()[m_size++] = v;
  } else {
    m_capacity = Common::downCast<size_type>(3 * (m_capacity + 1) / 2);

    auto data = std::make_unique<array_type>(m_capacity);
    std::copy(begin(), end(), data.get());

    m_data = std::move(data);

    m_data.get()[m_size++] = v;
  }
}

auto Frame::operator==(const Frame &other) const noexcept -> bool {
  return getPixelList().size() == other.getPixelList().size() &&
         getPixelList() == other.getPixelList();
}

void Frame::appendLayer(Attribute::GeometryValue layerId, const TextureTransparency8Frame &layer) {
  auto textureLayer = yuv444(layer.texture);
  const auto &transparencyLayer = layer.transparency;

  const auto &y_plane = textureLayer.getPlane(0);
  const auto &u_plane = textureLayer.getPlane(1);
  const auto &v_plane = textureLayer.getPlane(2);

  const auto &a_plane = transparencyLayer.getPlane(0);

  Common::parallelFor(y_plane.size(), [&](size_t k) {
    if (0 < a_plane[k]) {
      m_pixelList[k].push_back(Attribute{
          Attribute::TextureValue{y_plane[k], u_plane[k], v_plane[k]}, layerId, a_plane[k]});
    }
  });
}

auto Frame::getLayer(Attribute::GeometryValue layerId) const -> TextureTransparency8Frame {
  auto textureFrame = Common::Frame<>::yuv444(m_size, 10);
  auto transparencyFrame = Common::Frame<uint8_t>::lumaOnly(m_size);

  textureFrame.fillNeutral();

  Common::parallelFor(m_pixelList.size(), [&](size_t k) {
    const auto &pixel = m_pixelList[k];
    auto *const iter =
        std::lower_bound(pixel.begin(), pixel.end(), layerId,
                         [](auto pixel_, auto layerId_) { return pixel_.geometry < layerId_; });

    if (iter != pixel.end() && iter->geometry == layerId) {
      textureFrame.getPlane(0)[k] = iter->texture[0];
      textureFrame.getPlane(1)[k] = iter->texture[1];
      textureFrame.getPlane(2)[k] = iter->texture[2];

      transparencyFrame.getPlane(0)[k] = iter->transparency;
    }
  });

  return {yuv420(textureFrame), std::move(transparencyFrame)};
}
} // namespace TMIV::MpiPcs
