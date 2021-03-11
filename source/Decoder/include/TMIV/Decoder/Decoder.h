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

#ifndef TMIV_DECODER_DECODER_H
#define TMIV_DECODER_DECODER_H

#include <TMIV/Decoder/IDecoder.h>

#include <TMIV/Common/Json.h>
#include <TMIV/Decoder/EntityBasedPatchMapFilter.h>
#include <TMIV/Decoder/GeometryScaler.h>

namespace TMIV::Decoder {
class Decoder : public IDecoder {
private:
  GeometryScaler m_geometryScaler;
  EntityBasedPatchMapFilter m_entityBasedPatchMapFilter;

public:
  Decoder(const Common::Json &rootNode, const Common::Json & /* componentNode */);
  Decoder(const Decoder &) = delete;
  Decoder(Decoder &&) = default;
  auto operator=(const Decoder &) -> Decoder & = delete;
  auto operator=(Decoder &&) -> Decoder & = default;
  ~Decoder() override = default;

  void recoverFrame(MivBitstream::AccessUnit &frame) override;
};
} // namespace TMIV::Decoder

#endif
