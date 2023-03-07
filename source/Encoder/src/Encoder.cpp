/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#include "EncoderImpl.h"

namespace TMIV::Encoder {
Encoder::Encoder(const Common::Json &componentNode) : m_impl{new Impl{componentNode}} {}

Encoder::~Encoder() = default;

void Encoder::prepareSequence(const MivBitstream::SequenceConfig &sequenceConfig,
                              const Common::DeepFrameList &firstFrame) {
  m_impl->prepareSequence(sequenceConfig, firstFrame);
}

void Encoder::prepareAccessUnit() { m_impl->prepareAccessUnit(); }

void Encoder::pushFrame(Common::DeepFrameList sourceViews) {
  m_impl->pushFrame(std::move(sourceViews));
}

auto Encoder::completeAccessUnit() -> const EncoderParams & { return m_impl->completeAccessUnit(); }

auto Encoder::popAtlas() -> Common::V3cFrameList { return m_impl->popAtlas(); }

auto Encoder::maxLumaSamplesPerFrame() const -> size_t { return m_impl->maxLumaSamplesPerFrame(); }
} // namespace TMIV::Encoder
