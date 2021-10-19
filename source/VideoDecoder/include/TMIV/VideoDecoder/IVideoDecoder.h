/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#ifndef TMIV_VIDEODECODER_IVIDEODECODER_H
#define TMIV_VIDEODECODER_IVIDEODECODER_H

#include <TMIV/Common/Frame.h>

#include <TMIV/MivBitstream/V3cParameterSet.h>

#include <functional>
#include <memory>

namespace TMIV::VideoDecoder {
// A NAL unit source is a function that returns NAL units as a string (blob of bytes). An empty
// string indicates that thare are no more NAL units.
using NalUnitSource = std::function<std::string()>;

class IVideoDecoder {
public:
  IVideoDecoder() = default;
  IVideoDecoder(const IVideoDecoder &) = delete;
  IVideoDecoder(IVideoDecoder &&) = default;
  auto operator=(const IVideoDecoder &) -> IVideoDecoder & = delete;
  auto operator=(IVideoDecoder &&) -> IVideoDecoder & = default;
  virtual ~IVideoDecoder() = default;

  // Get the next frame in picture order. If there are no more frames then the result will be empty.
  // Side-effects may be that NAL units may be taken from the source and there may be screen output.
  virtual auto getFrame() -> std::unique_ptr<Common::Frame<>> = 0;
};
} // namespace TMIV::VideoDecoder

#endif
