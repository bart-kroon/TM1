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
#include <TMIV/Common/Matrix.h>
#include <TMIV/Metadata/IvSequenceParams.h>

namespace TMIV::Image {
// The maximum level for an unsigned integer of the specified number of bits
constexpr unsigned maxLevel(unsigned bits);

// Modify depth range from 16-bit to 10-bit with camera parameters that may have different
// normDispRange and depthOccMapThreshold values.
auto modifyDepthRange(const Common::MVD16Frame &frame16,
                      const Metadata::CameraParametersList &cameras16,
                      const Metadata::CameraParametersList &cameras10) -> Common::MVD10Frame;

// Expand a YUV 4:2:0 10-bit texture to packed 4:4:4 32-bit float texture with
// linear transfer and nearest interpolation for chroma
Common::Mat<Common::Vec3f> expandTexture(const Common::Frame<Common::YUV420P10> &inYuv);

// Quantize a packed 4:4:4 32-bit float texture as YUV 4:2:0 10-bit texture with
// linear transfer and area interpolation for chroma
Common::Frame<Common::YUV444P10> quantizeTexture(const Common::Mat<Common::Vec3f> &in);

// Expand a 10-bit depth value. The return value is 0.F for levels below depthOccMapThreshold. Other
// levels are translated to depth in meter.
float expandDepthValue10(const Metadata::CameraParameters &camera, uint16_t x);

// Expand a 16-bit depth value. The return value is 0.F for levels below depthOccMapThreshold. Other
// levels are translated to depth in meter.
float expandDepthValue16(const Metadata::CameraParameters &camera, uint16_t x);

// Expand a 10-bit depth map. Levels below depthOccMapThreshold become 0.F. Other levels are
// translated to depth in meter.
Common::Mat<float> expandDepth(const Metadata::CameraParameters &camera,
                               const Common::Depth10Frame &in);

// Expand a 10-bit depth map. Levels below depthOccMapThreshold become 0.F. Other levels are
// translated to depth in meter.
Common::Mat<float> expandDepth(const Metadata::CameraParameters &camera,
                               const Common::Depth16Frame &in);

// Quantize a normalized disparity map (in meter^-1) to 16-bit values according to the normDispRange
// and depthOccMapThreshold camera parameters
Common::Depth16Frame quantizeNormDisp16(const Metadata::CameraParameters &camera,
                                        const Common::Mat<float> &in);
} // namespace TMIV::Image

#include "Image.hpp"

#endif
