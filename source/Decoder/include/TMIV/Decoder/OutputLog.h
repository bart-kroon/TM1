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

#ifndef TMIV_DECODER_OUTPUTLOG_H
#define TMIV_DECODER_OUTPUTLOG_H

#include <TMIV/MivBitstream/AccessUnit.h>

namespace TMIV::Decoder {
class HashFunction {
public:
  template <typename ConvertibleToUint32,
            typename = std::enable_if_t<std::is_integral_v<ConvertibleToUint32> ||
                                        std::is_enum_v<ConvertibleToUint32>>>
  constexpr auto consume(ConvertibleToUint32 value) noexcept -> HashFunction &;

  constexpr auto consume(MivBitstream::ViewId value) noexcept -> HashFunction &;
  auto consumeF(float value) noexcept -> HashFunction &;
  [[nodiscard]] constexpr auto result() const noexcept;
  [[nodiscard]] static auto toString(uint32_t value) -> std::string;

  using Result = uint32_t;

private:
  uint32_t m_hash{0xFFFFFFFF};
};

[[nodiscard]] auto videoDataHash(const Common::Frame<> &frame) noexcept -> HashFunction::Result;
[[nodiscard]] auto blockToPatchMapHash(const MivBitstream::AtlasAccessUnit &frame) noexcept
    -> HashFunction::Result;
[[nodiscard]] auto patchParamsListHash(const MivBitstream::PatchParamsList &ppl) noexcept
    -> HashFunction::Result;
[[nodiscard]] auto viewParamsListHash(const MivBitstream::ViewParamsList &vpl) noexcept
    -> HashFunction::Result;
[[nodiscard]] auto asmeHash(const MivBitstream::AtlasAccessUnit &frame) noexcept
    -> HashFunction::Result;
[[nodiscard]] auto afmeHash(const MivBitstream::AtlasAccessUnit &frame) noexcept
    -> HashFunction::Result;
[[nodiscard]] auto casmeHash(const MivBitstream::AccessUnit &frame) noexcept
    -> HashFunction::Result;
void writeFrameToOutputLog(const MivBitstream::AccessUnit &frame, std::ostream &stream);
} // namespace TMIV::Decoder

#endif
