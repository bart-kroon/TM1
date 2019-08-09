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

#ifndef _TMIV_IMAGE_IMAGE_H_
#define _TMIV_IMAGE_IMAGE_H_

#include <TMIV/Common/Frame.h>
#include <TMIV/Metadata/CameraParametersList.h>

namespace TMIV::Image {
// The maximum level for an unsigned integer of the specified number of bits
constexpr unsigned maxLevel(unsigned bits);

// Expand value with levels 0..2^bits-1 to a value in [0, 1] range with linear
// transfer
template <unsigned bits> float expandValue(uint16_t x);

// Expand a 10/16-bit depth value to a 32-bit float matrix with depth
// values in meters. Input level 0 indicates invalid depth and is mapped to NaN.
template <unsigned bits>
float expandDepthValue(const Metadata::CameraParameters &camera, uint16_t x);

// Expand a YUV 4:0:0 10/16-bit depth map to a 32-bit float matrix with depth
// values in meters. Input level 0 indicates invalid depth and is mapped to NaN.
Common::Mat<float> expandDepth(const Metadata::CameraParameters &camera,
                               const Common::Frame<Common::YUV400P10> &inYuv);

// Quantize a value in the [0, 1] range to levels 0..2^bits - 1
// NaN values are assigend to level 0
// Other values are clipped to the [0, 1] range
template <unsigned bits> uint16_t quantizeValue(float x);

// Expand a YUV 4:2:0 10-bit texture to packed 4:4:4 32-bit float texture with
// linear transfer and nearest interpolation for chroma
Common::Mat<Common::Vec3f>
expandTexture(const Common::Frame<Common::YUV420P10> &inYuv);

// Quantize a packed 4:4:4 32-bit float texture as YUV 4:2:0 10-bit texture with
// linear transfer and area interpolation for chroma
Common::Frame<Common::YUV444P10>
quantizeTexture(const Common::Mat<Common::Vec3f> &in);

// Expand a YUV 4:0:0 10/16-bit depth map to a 32-bit float matrix with depth
// values in meters. Input level 0 indicates invalid depth and is mapped to NaN.
Common::Mat<float> expandDepth(const Metadata::CameraParameters &camera,
                               const Common::Frame<Common::YUV400P16> &inYuv);

// Quantize a 32-bit float depth map with depth values in diopters. NaN values
// are translated to level 0.
Common::Frame<Common::YUV400P10>
quantizeNormDisp10(const Metadata::CameraParameters &camera,
                   const Common::Mat<float> &in);
Common::Frame<Common::YUV400P16>
quantizeNormDisp16(const Metadata::CameraParameters &camera,
                   const Common::Mat<float> &in);

// Quantize a 32-bit float depth map with depth values in meters. NaN values
// are translated to level 0.
Common::Frame<Common::YUV400P10>
quantizeDepth10(const Metadata::CameraParameters &camera,
                const Common::Mat<float> &in);
Common::Frame<Common::YUV400P16>
quantizeDepth16(const Metadata::CameraParameters &camera,
                const Common::Mat<float> &in);

template <typename ToInt, typename WorkInt>
auto compressRangeValue(WorkInt x, WorkInt fromBits, WorkInt toBits,
                        WorkInt offsetMax) -> ToInt;

template <typename OutFormat, typename InFormat>
auto compressDepthRange(
    const Common::Frame<InFormat> &frame, unsigned offsetMax,
    unsigned bits = Common::detail::PixelFormatHelper<InFormat>::bitDepth)
    -> Common::Frame<OutFormat>;

template <typename OutFormat, typename InFormat>
auto compressDepthRange(
    const Common::MVDFrame<InFormat> &frame, unsigned offsetMax,
    unsigned bits = Common::detail::PixelFormatHelper<InFormat>::bitDepth)
    -> Common::MVDFrame<OutFormat>;

template <typename ToInt, typename WorkInt>
auto decompressRangeValue(WorkInt x, WorkInt fromBits, WorkInt toBits,
                          WorkInt offsetMax) -> ToInt;

template <typename OutFormat, typename InFormat>
auto decompressDepthRange(
    const Common::Frame<InFormat> &frame, unsigned offsetMax,
    unsigned bits = Common::detail::PixelFormatHelper<InFormat>::bitDepth)
    -> Common::Frame<OutFormat>;

template <typename OutFormat, typename InFormat>
auto decompressDepthRange(
    const Common::MVDFrame<InFormat> &frame, unsigned offsetMax,
    unsigned bits = Common::detail::PixelFormatHelper<InFormat>::bitDepth)
    -> Common::MVDFrame<OutFormat>;

// Requantize a value
//
//  Both input and output types have to be unsigned integers. The input type has
//  to be wide enough to multiply the maximum input and output with each other.
template <typename ToInt, typename WorkInt>
auto requantizeValue(WorkInt x, WorkInt fromBits, WorkInt toBits) -> ToInt;

// Requantize a frame
//
// The optional second parameter allows to specify a different number of input
// bits than the underlying type, e.g. 10-bit values stored in 16-bit types.
//
// This function can also do YUV 4:x:y format conversions but no chroma scaling
template <typename OutFormat, typename InFormat>
auto requantize(
    const Common::Frame<InFormat> &frame,
    unsigned bits = Common::detail::PixelFormatHelper<InFormat>::bitDepth)
    -> Common::Frame<OutFormat>;

// Requantize the depth maps of a multiview frame
//
// The optional second parameter allows to specify a different number of input
// bits than the underlying type, e.g. 10-bit values stored in 16-bit types.
//
// This function can also do YUV 4:x:y format conversions but no chroma scaling
template <typename OutFormat, typename InFormat>
auto requantize(
    const Common::MVDFrame<InFormat> &frame,
    unsigned bits = Common::detail::PixelFormatHelper<InFormat>::bitDepth)
    -> Common::MVDFrame<OutFormat>;
} // namespace TMIV::Image

#include "Image.hpp"

#endif
