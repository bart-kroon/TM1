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

#include <TMIV/Common/Factory.h>

using TMIV::Aggregator::IAggregator;
using TMIV::Packer::IPacker;
using TMIV::Pruner::IPruner;

namespace TMIV::Encoder {
Encoder::Impl::Impl(const Common::Json &componentNode)
    : m_pruner{Common::create<Pruner::IPruner>("Pruner", componentNode, componentNode)}
    , m_aggregator{Common::create<IAggregator>("Aggregator", componentNode, componentNode)}
    , m_packer{Common::create<IPacker>("Packer", componentNode, componentNode)}
    , m_config(componentNode) {}

auto Encoder::Impl::maxLumaSamplesPerFrame() const -> size_t { return m_maxLumaSamplesPerFrame; }

auto Encoder::Impl::isStart(const SourceUnit & /* unit */) -> bool {
  return ++m_lastIdx % m_config.interPeriod == 0;
}

void Encoder::Impl::process(std::vector<SourceUnit> buffer,
                            const Common::StageSource<CodableUnit> &source_) {
  if (m_firstIdx == 0) {
    prepareSequence(buffer.front());
  }

  m_params.foc = m_firstIdx % m_config.intraPeriod;

  prepareAccessUnit();

  for (auto &sourceUnit : buffer) {
    pushFrame(std::move(sourceUnit.deepFrameList));
  }

  completeAccessUnit();
  constructVideoFrames();

  auto type = m_firstIdx % m_config.intraPeriod == 0 ? MivBitstream::CodableUnitType::IDR
                                                     : MivBitstream::CodableUnitType::TRIAL;

  for (auto &deepFrameList : m_videoFrameBuffer) {
    source_.encode({m_params, std::move(deepFrameList), type});

    type = MivBitstream::CodableUnitType::SKIP;
    // Pattern for intraPeriod=8 and interPeriod=4:
    //    IDR SKIP SKIP SKIP TRIAL SKIP SKIP SKIP IDR ...
    // FOC=0   1    2    3    4     5    6    7    0  ...
  }

  m_videoFrameBuffer.clear();

  m_firstIdx = m_lastIdx;
}
} // namespace TMIV::Encoder
