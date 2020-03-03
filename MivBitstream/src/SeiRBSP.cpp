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

#include <TMIV/MivBitstream/SeiRBSP.h>

#include <TMIV/Common/Bitstream.h>

#include "verify.h"

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto operator<<(std::ostream &stream, const SeiMessage & /* x */) -> std::ostream & {
  // TODO(BK): Implement SeiRBSP
  return stream;
}

auto SeiMessage::operator==(const SeiMessage & /* other */) const noexcept -> bool {
  // TODO(BK): Implement SeiRBSP
  return true;
}

auto SeiMessage::operator!=(const SeiMessage &other) const noexcept -> bool {
  return !operator==(other);
}

auto SeiMessage::decodeFrom(Common::InputBitstream &bitstream) -> SeiMessage {
  // TODO(BK): Implement SeiRBSP
  bitstream.getFlag();
  return {};
}

void SeiMessage::encodeTo(Common::OutputBitstream &bitstream) {
  // TODO(BK): Implement SeiRBSP
  bitstream.putFlag(false);
}

SeiRBSP::SeiRBSP(vector<SeiMessage> messages) : m_messages{move(messages)} {}

auto operator<<(ostream &stream, const SeiRBSP &x) -> ostream & {
  for (const auto &x : x.messages()) {
    stream << x;
  }
  return stream;
}

auto SeiRBSP::operator==(const SeiRBSP &other) const noexcept -> bool {
  return messages() == other.messages();
}

auto SeiRBSP::operator!=(const SeiRBSP &other) const noexcept -> bool { return !operator==(other); }

auto SeiRBSP::decodeFrom(istream &stream) -> SeiRBSP {
  auto messages = vector<SeiMessage>{};
  InputBitstream bitstream{stream};

  do {
    messages.push_back(SeiMessage::decodeFrom(bitstream));
  } while (bitstream.moreRbspData());
  bitstream.rbspTrailingBits();

  return SeiRBSP{messages};
}

void SeiRBSP::encodeTo(ostream &stream) const {
  OutputBitstream bitstream{stream};

  VERIFY_MIVBITSTREAM(!messages().empty());

  for (const auto &x : messages()) {
    x.encodeTo(bitstream);
  }
  bitstream.rbspTrailingBits();
}
} // namespace TMIV::MivBitstream
