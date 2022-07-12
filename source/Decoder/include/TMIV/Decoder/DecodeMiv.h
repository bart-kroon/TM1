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

#ifndef TMIV_DECODER_MIVDECODER_H
#define TMIV_DECODER_MIVDECODER_H

#include "DecodeAtlas.h"
#include "DecodeCommonAtlas.h"

#include <TMIV/Common/Decoder.h>
#include <TMIV/MivBitstream/AccessUnit.h>

namespace TMIV::Decoder {
using VideoDecoderFactory =
    Common::DecoderFactory<MivBitstream::VideoSubBitstream, Common::DecodedFrame,
                           const MivBitstream::V3cParameterSet &, MivBitstream::V3cUnitHeader>;

using CommonAtlasDecoderFactory =
    Common::DecoderFactory<MivBitstream::AtlasSubBitstream, CommonAtlasAccessUnit,
                           const MivBitstream::V3cParameterSet &>;

using AtlasDecoderFactory =
    Common::DecoderFactory<MivBitstream::AtlasSubBitstream, AtlasAccessUnit,
                           const MivBitstream::V3cParameterSet &, MivBitstream::V3cUnitHeader>;

auto decodeMiv(Common::Source<MivBitstream::V3cUnit> source,
               VideoDecoderFactory videoDecoderFactory, PtlChecker::SharedChecker checker,
               CommonAtlasDecoderFactory commonAtlasDecoderFactory,
               AtlasDecoderFactory atlasDecoderFactory) -> Common::Source<MivBitstream::AccessUnit>;

enum class ErrorCode : int32_t {
  expected_atlas_to_be_irap,
  expected_common_atlas_to_be_irap,
  expected_miv_extension,
  expected_video,
  expected_vps,
  misaligned_atlas_foc,
  misaligned_common_atlas_foc,
  misaligned_video_irap,
  missing_atlas_irap,
  missing_common_atlas_irap,
  unsupported_vps_extension
};

[[nodiscard]] auto errorStringFor(ErrorCode code) -> const char *;

class Exception : public Common::MivBitstreamError {
public:
  Exception(ErrorCode code, char const *file, int32_t line);

  [[nodiscard]] auto code() const -> ErrorCode { return m_code; }
  [[nodiscard]] auto file() const -> char const * { return m_file; }
  [[nodiscard]] auto line() const -> int32_t { return m_line; }

private:
  ErrorCode m_code;
  char const *m_file = nullptr;
  int32_t m_line = 0;
};
} // namespace TMIV::Decoder

#endif
