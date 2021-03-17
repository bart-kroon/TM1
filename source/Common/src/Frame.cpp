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

#include <TMIV/Common/Frame.h>

#include <TMIV/Common/Bytestream.h>
#include <TMIV/Common/Common.h>
#include <TMIV/Common/Thread.h>

#include <sstream>

namespace TMIV::Common {
namespace {
template <class TO, class FROM> auto yuv420p_impl(const Frame<FROM> &frame) -> Frame<TO> {
  Frame<TO> result(frame.getWidth(), frame.getHeight());
  std::copy(std::begin(frame.getPlane(0)), std::end(frame.getPlane(0)),
            std::begin(result.getPlane(0)));

  PRECONDITION(frame.getWidth() % 2 == 0 && frame.getHeight() % 2 == 0);
  const int rows = result.getHeight() / 2;
  const int cols = result.getWidth() / 2;

  for (int k = 1; k < 3; ++k) {
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        auto sum = frame.getPlane(k)(2 * i, 2 * j) + frame.getPlane(k)(2 * i + 1, 2 * j) +
                   frame.getPlane(k)(2 * i, 2 * j + 1) + frame.getPlane(k)(2 * i + 1, 2 * j + 1);
        using base_type = typename detail::PixelFormatHelper<TO>::base_type;
        result.getPlane(k)(i, j) = static_cast<base_type>((sum + 2) / 4);
      }
    }
  }

  return result;
}

template <class TO, class FROM> auto yuv444p_impl(const Frame<FROM> &frame) -> Frame<TO> {
  PRECONDITION(frame.getWidth() % 2 == 0 && frame.getHeight() % 2 == 0);

  auto result = Frame<TO>{frame.getWidth(), frame.getHeight()};

  for (int i = 0; i < frame.getHeight(); ++i) {
    for (int j = 0; j < frame.getWidth(); ++j) {
      result.getPlane(0)(i, j) = frame.getPlane(0)(i, j);
      result.getPlane(1)(i, j) = frame.getPlane(1)(i / 2, j / 2);
      result.getPlane(2)(i, j) = frame.getPlane(2)(i / 2, j / 2);
    }
  }

  return result;
}
} // namespace

auto yuv420p(const Frame<YUV444P8> &frame) -> Frame<YUV420P8> {
  return yuv420p_impl<YUV420P8>(frame);
}

auto yuv420p(const Frame<YUV444P10> &frame) -> Frame<YUV420P10> {
  return yuv420p_impl<YUV420P10>(frame);
}

auto yuv420p(const Frame<YUV444P16> &frame) -> Frame<YUV420P16> {
  return yuv420p_impl<YUV420P16>(frame);
}

auto yuv444p(const Frame<YUV420P8> &frame) -> Frame<YUV444P8> {
  return yuv444p_impl<YUV444P8>(frame);
}

auto yuv444p(const Frame<YUV420P10> &frame) -> Frame<YUV444P10> {
  return yuv444p_impl<YUV444P10>(frame);
}

auto yuv444p(const Frame<YUV420P16> &frame) -> Frame<YUV444P16> {
  return yuv444p_impl<YUV444P16>(frame);
}

auto expandTexture(const Frame<YUV444P10> &inYuv) -> Mat<Vec3f> {
  const auto &Y = inYuv.getPlane(0);
  const auto &U = inYuv.getPlane(1);
  const auto &V = inYuv.getPlane(2);
  Mat<Vec3f> out(inYuv.getPlane(0).sizes());
  const auto width = Y.width();
  const auto height = Y.height();
  constexpr auto bitDepth = 10U;

  for (unsigned i = 0; i != height; ++i) {
    for (unsigned j = 0; j != width; ++j) {
      out(i, j) = Vec3f{expandValue(Y(i, j), bitDepth), expandValue(U(i, j), bitDepth),
                        expandValue(V(i, j), bitDepth)};
    }
  }
  return out;
}

auto expandLuma(const Frame<YUV420P10> &inYuv) -> Mat<float> {
  auto out = Mat<float>(inYuv.getPlane(0).sizes());
  std::transform(inYuv.getPlane(0).cbegin(), inYuv.getPlane(0).cend(), out.begin(), [](auto value) {
    constexpr auto bitDepth = 10U;
    return expandValue(value, bitDepth);
  });
  return out;
}

auto quantizeTexture(const Mat<Vec3f> &in) -> Frame<YUV444P10> {
  Frame<YUV444P10> outYuv(static_cast<int>(in.width()), static_cast<int>(in.height()));
  const auto width = in.width();
  const auto height = in.height();

  for (int k = 0; k < 3; ++k) {
    for (unsigned i = 0; i != height; ++i) {
      for (unsigned j = 0; j != width; ++j) {
        constexpr auto bitDepth = 10U;
        outYuv.getPlane(k)(i, j) = quantizeValue<uint16_t>(in(i, j)[k], bitDepth);
      }
    }
  }

  return outYuv;
}

namespace MpiPcs {
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
  assert(blob.size() == buffer.size());

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
    m_capacity = 3 * (m_capacity + 1) / 2;

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

void Frame::appendLayer(Attribute::Geometry layerId, const TextureTransparency8Frame &layer) {
  auto textureLayer = yuv444p(layer.texture);
  const auto &transparencyLayer = layer.transparency;

  const auto &y_plane = textureLayer.getPlane(0);
  const auto &u_plane = textureLayer.getPlane(1);
  const auto &v_plane = textureLayer.getPlane(2);

  const auto &a_plane = transparencyLayer.getPlane(0);

  parallel_for(y_plane.size(), [&](std::size_t k) {
    if (0 < a_plane[k]) {
      m_pixelList[k].push_back(
          Attribute{Attribute::Texture{y_plane[k], u_plane[k], v_plane[k]}, layerId, a_plane[k]});
    }
  });
}

auto Frame::getLayer(Attribute::Geometry layerId) const -> TextureTransparency8Frame {
  Texture444Frame textureFrame{m_size.x(), m_size.y()};
  Transparency8Frame transparencyFrame{m_size.x(), m_size.y()};

  textureFrame.fillNeutral();

  parallel_for(m_pixelList.size(), [&](std::size_t k) {
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

  return {yuv420p(textureFrame), std::move(transparencyFrame)};
}

} // namespace MpiPcs

} // namespace TMIV::Common
