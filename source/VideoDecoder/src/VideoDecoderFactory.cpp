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

#include <TMIV/VideoDecoder/VideoDecoderFactory.h>

#if HAVE_HM
#include <TMIV/VideoDecoder/HmVideoDecoder.h>
#endif

#if HAVE_VVDEC
#include <TMIV/VideoDecoder/VVdeCVideoDecoder.h>
#endif

#include <fmt/ostream.h>

namespace TMIV::VideoDecoder {
auto operator<<(std::ostream &stream, DecoderId x) -> std::ostream & {
  switch (x) {
  case DecoderId::AVC_Progressive_High:
    return stream << "AVC Progressive High";
  case DecoderId::HEVC_Main10:
    return stream << "HEVC Main10";
  case DecoderId::HEVC444:
    return stream << "HEVC444";
  case DecoderId::VVC_Main10:
    return stream << "VVC Main10";
  default:
    return stream << "[unknown:" << static_cast<int32_t>(x) << "]";
  }
}

auto create(NalUnitSource source, DecoderId decoderId) -> std::unique_ptr<VideoDecoderBase> {
#if HAVE_HM
  if (decoderId == DecoderId::HEVC_Main10) {
    return std::make_unique<HmVideoDecoder>(std::move(source));
  }
#endif
#if HAVE_VVDEC
  if (decoderId == DecoderId::VVC_Main10) {
    return std::make_unique<VVdeCVideoDecoder>(std::move(source));
  }
#endif
  throw std::runtime_error(fmt::format("There is no built-in support for the {} codec", decoderId));
}
} // namespace TMIV::VideoDecoder
