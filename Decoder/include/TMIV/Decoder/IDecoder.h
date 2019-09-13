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

#ifndef _TMIV_DECODER_IDECODER_H_
#define _TMIV_DECODER_IDECODER_H_

#include <TMIV/Common/Frame.h>
#include <TMIV/Metadata/AtlasParametersList.h>
#include <TMIV/Metadata/CameraParametersList.h>

namespace TMIV::Decoder {
class IDecoder {
public:
  IDecoder() = default;
  IDecoder(const IDecoder &) = delete;
  IDecoder(IDecoder &&) = default;
  IDecoder &operator=(const IDecoder &) = delete;
  IDecoder &operator=(IDecoder &&) = default;
  virtual ~IDecoder() = default;

  // Change the atlas size.
  //
  // This call invalidates the patch list and requires updatePatchList to be
  // called before any call to decodeFrame.
  virtual void updateAtlasSize(std::vector<Common::Vec2i> sizes) = 0;

  // Change the patch list.
  //
  // This call shall be preceded by at least one call to updateAtlasSize.
  virtual void updatePatchList(Metadata::AtlasParametersList patches,
                               const Common::MVD16Frame &frame) = 0;

  // Change the camera list.
  virtual void updateCameraList(Metadata::CameraParametersList cameras) = 0;

  // Decode a frame and render to a target viewport.
  //
  // This call shall be preceded by at least one call of each of
  // updateAtlasSize, updatePatchList and updateCameraList.
  virtual Common::Texture444Depth16Frame
  decodeFrame(Common::MVD16Frame atlas, const Metadata::CameraParameters &target) const = 0;
};
} // namespace TMIV::Decoder

#endif
