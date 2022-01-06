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

#ifndef TMIV_COMMON_FRAME_H
#error "Include the .h instead of the .hpp."
#endif

#include <TMIV/Common/verify.h>

#include <cstring>

namespace TMIV::Common {
template <typename Element>
void Frame<Element>::create(Vec2i size, uint32_t bitDepth, ColorFormat colorFormat) {
  PRECONDITION(bitDepth <= maxBitDepth);
  m_bitDepth = bitDepth;

  if (!empty() && size == getSize()) {
    fillZero();
  } else {
    const auto rows = static_cast<size_t>(size.y());
    const auto columns = static_cast<size_t>(size.x());

    if (colorFormat == ColorFormat::YUV400) {
      m_planes.resize(1);
      m_planes[0].resize(rows, columns);
    }
    if (colorFormat == ColorFormat::YUV420) {
      PRECONDITION(rows % 2 == 0 && columns % 2 == 0);
      m_planes.resize(3);
      m_planes[0].resize(rows, columns);
      m_planes[1].resize(rows / 2, columns / 2);
      m_planes[2].resize(rows / 2, columns / 2);
    }
    if (colorFormat == ColorFormat::YUV444) {
      m_planes.resize(3);
      m_planes[0].resize(rows, columns);
      m_planes[1].resize(rows, columns);
      m_planes[2].resize(rows, columns);
    }
  }
}

template <typename Element> auto Frame<Element>::getColorFormat() const noexcept -> ColorFormat {
  PRECONDITION(!empty());

  if (m_planes.size() == 1) {
    return ColorFormat::YUV400;
  }
  if (m_planes[1].size(0) < m_planes[0].size(0)) {
    return ColorFormat::YUV420;
  }
  return ColorFormat::YUV444;
}

template <typename Element> auto Frame<Element>::getByteCount() const noexcept {
  switch (getColorFormat()) {
  case ColorFormat::YUV400:
    return m_planes.front().size() * sizeof(Element);
  case ColorFormat::YUV420:
    return (m_planes.front().size() * sizeof(Element) * 3) / 2;
  case ColorFormat::YUV444:
    return m_planes.front().size() * sizeof(Element) * 3;
  default:
    UNREACHABLE;
  }
}

template <typename Element> void Frame<Element>::readFrom(std::istream &stream) {
  for (auto &plane : m_planes) {
    auto buffer = std::vector<char>(plane.size() * sizeof(Element));
    stream.read(buffer.data(), assertDownCast<std::streamsize>(buffer.size()));
    std::memcpy(plane.data(), buffer.data(), buffer.size());
  }
}

template <typename Element> void Frame<Element>::writeTo(std::ostream &stream) const {
  for (const auto &plane : m_planes) {
    auto buffer = std::vector<char>(plane.size() * sizeof(Element));
    std::memcpy(buffer.data(), plane.data(), buffer.size());
    stream.write(buffer.data(), assertDownCast<std::streamsize>(buffer.size()));
  }
}

template <typename Element> void Frame<Element>::fillValue(Element value) noexcept {
  for (auto &plane : m_planes) {
    std::fill(plane.begin(), plane.end(), value);
  }
}

template <typename Element>
template <typename OtherElement>
void Frame<Element>::fillInvalid(const Frame<OtherElement> &mask, Element value) noexcept {
  PRECONDITION(!mask.empty());

  for (auto &plane : m_planes) {
    PRECONDITION(plane.sizes() == mask.getPlane(0).sizes());

    auto i_mask = mask.getPlane(0).cbegin();

    for (auto &sample : plane) {
      if (*i_mask++ == 0) {
        sample = value;
      }
    }
  }
}

template <typename Element>
template <typename OtherElement>
void Frame<Element>::fillInvalidWithNeutral(const Frame<OtherElement> &mask) noexcept {
  fillInvalid(mask, neutralValue());
}

template <typename Element> auto Frame<Element>::setBitDepth(uint32_t value) noexcept {
  PRECONDITION(value <= maxBitDepth);
  m_bitDepth = value;
}

namespace detail {
template <typename Element> void octaveDownArea(const Mat<Element> &from, Mat<Element> &to) {
  using UInt = std::conditional_t<std::numeric_limits<Element>::digits <= 16, uint32_t, uint64_t>;

  const auto rows = from.size(0) / 2;
  const auto cols = from.size(1) / 2;
  to.resize(std::array{rows, cols});

  for (size_t i = 0; i < rows; ++i) {
    for (size_t j = 0; j < cols; ++j) {
      to(i, j) = static_cast<Element>((UInt{from(2 * i, 2 * j)} +         //
                                       UInt{from(2 * i + 1, 2 * j)} +     //
                                       UInt{from(2 * i, 2 * j + 1)} +     //
                                       UInt{from(2 * i + 1, 2 * j + 1)} + //
                                       UInt{2}) /
                                      UInt{4});
    }
  }
}

template <typename Element> void octaveUpNearest(const Mat<Element> &from, Mat<Element> &to) {
  const auto rows = from.size(0) * 2;
  const auto cols = from.size(1) * 2;
  to.resize(std::array{rows, cols});

  for (size_t i = 0; i < rows; ++i) {
    for (size_t j = 0; j < cols; ++j) {
      to(i, j) = from(i / 2, j / 2);
    }
  }
}
} // namespace detail

template <typename Element> auto yuv400(const Frame<Element> &frame) {
  if (frame.empty() || frame.getColorFormat() == ColorFormat::YUV400) {
    return frame;
  }

  auto result = Frame<Element>{};
  result.setBitDepth(frame.getBitDepth());
  result.getPlanes().resize(1);
  result.getPlane(0) = frame.getPlane(0);
  return result;
}

template <typename Element> auto yuv420(const Frame<Element> &frame) {
  if (frame.empty() || frame.getColorFormat() == ColorFormat::YUV420) {
    return frame;
  }

  auto result = Frame<Element>{};
  result.setBitDepth(frame.getBitDepth());
  result.getPlanes().resize(3);
  result.getPlane(0) = frame.getPlane(0);

  if (frame.getColorFormat() == ColorFormat::YUV400) {
    const auto rows = frame.getPlane(0).size(0);
    const auto cols = frame.getPlane(0).size(1);
    PRECONDITION(rows % 2 == 0 && cols % 2 == 0);

    result.getPlane(1) = Mat<Element>{std::array{rows / 2, cols / 2}, frame.neutralValue()};
    result.getPlane(2) = Mat<Element>{std::array{rows / 2, cols / 2}, frame.neutralValue()};
    return result;
  }

  if (frame.getColorFormat() == ColorFormat::YUV444) {
    detail::octaveDownArea(frame.getPlane(1), result.getPlane(1));
    detail::octaveDownArea(frame.getPlane(2), result.getPlane(2));
    return result;
  }

  UNREACHABLE;
}

template <typename Element> auto yuv444(const Frame<Element> &frame) {
  if (frame.empty() || frame.getColorFormat() == ColorFormat::YUV444) {
    return frame;
  }

  auto result = Frame<Element>{};
  result.setBitDepth(frame.getBitDepth());
  result.getPlanes().resize(3);
  result.getPlane(0) = frame.getPlane(0);

  if (frame.getColorFormat() == ColorFormat::YUV400) {
    result.getPlane(1) = Mat<Element>{frame.getPlane(0).sizes(), frame.neutralValue()};
    result.getPlane(2) = Mat<Element>{frame.getPlane(0).sizes(), frame.neutralValue()};
    return result;
  }

  if (frame.getColorFormat() == ColorFormat::YUV420) {
    detail::octaveUpNearest(frame.getPlane(1), result.getPlane(1));
    detail::octaveUpNearest(frame.getPlane(2), result.getPlane(2));
    return result;
  }

  UNREACHABLE;
}

template <typename OtherElement, typename Element> auto elementCast(const Frame<Element> &inFrame) {
  static_assert(!std::is_same_v<OtherElement, Element>);

  auto outFrame =
      Frame<OtherElement>{inFrame.getSize(), inFrame.getBitDepth(), inFrame.getColorFormat()};
  auto outPlane = outFrame.getPlanes().begin();

  for (const auto &inPlane : inFrame.getPlanes()) {
    std::transform(inPlane.cbegin(), inPlane.cend(), (*outPlane++).begin(),
                   [](auto sample) { return Common::assertDownCast<OtherElement>(sample); });
  }

  return outFrame;
}
} // namespace TMIV::Common
