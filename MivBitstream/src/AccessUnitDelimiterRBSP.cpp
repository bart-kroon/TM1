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

#include <TMIV/MivBitstream/AccessUnitDelimiterRBSP.h>

#include "verify.h"

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
std::ostream &operator<<(std::ostream &stream, AframeType x) {
  switch (x) {
  case AframeType::I:
    return stream << "I_TILE_GRP";
  case AframeType::P_and_I:
    return stream << "P_TILE_GRP and I_TILE_GRP";
  case AframeType::SKIP_P_and_I:
    return stream << "SKIP_TILE_GRP, P_TILE_GRP and I_TILE_GRP";
  case AframeType::SKIP:
    return stream << "SKIP_TILE_GRP";
  default:
    return stream << "[unknown:" << int(x) << "]";
  }
}

auto operator<<(std::ostream &stream, const AccessUnitDelimiterRBSP &x) -> std::ostream & {
  return stream << "aframe_type=" << x.aframe_type() << '\n';
}

auto AccessUnitDelimiterRBSP::decodeFrom(std::istream &stream) -> AccessUnitDelimiterRBSP {
  InputBitstream bitstream{stream};

  auto aframe_type = AframeType(bitstream.readBits(3));
  VERIFY_VPCCBITSTREAM(AframeType::I <= aframe_type && aframe_type <= AframeType::SKIP);

  bitstream.rbspTrailingBits();

  return AccessUnitDelimiterRBSP{aframe_type};
}

void AccessUnitDelimiterRBSP::encodeTo(std::ostream &stream) const {
  VERIFY_VPCCBITSTREAM(AframeType::I <= aframe_type() && aframe_type() <= AframeType::SKIP);

  OutputBitstream bitstream{stream};
  bitstream.writeBits(unsigned(aframe_type()), 3);

  bitstream.rbspTrailingBits();
}
} // namespace TMIV::MivBitstream