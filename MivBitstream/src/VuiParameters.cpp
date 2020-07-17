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

#include <TMIV/MivBitstream/VuiParameters.h>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto operator<<(ostream &stream, const CoordinateAxisSystemParams &x) -> ostream & {
  stream << "cas_forward_axis=" << int(x.cas_forward_axis()) << '\n';
  stream << "cas_delta_left_axis_minus1=" << int(x.cas_delta_left_axis_minus1()) << '\n';
  stream << "cas_forward_sign=" << boolalpha << x.cas_forward_sign() << '\n';
  stream << "cas_left_sign=" << boolalpha << x.cas_left_sign() << '\n';
  stream << "cas_up_sign=" << boolalpha << x.cas_up_sign() << '\n';
  return stream;
}

auto CoordinateAxisSystemParams::decodeFrom(InputBitstream &bitstream)
    -> CoordinateAxisSystemParams {
  auto x = CoordinateAxisSystemParams{};

  x.cas_forward_axis(bitstream.readBits<uint8_t>(2))
      .cas_delta_left_axis_minus1(bitstream.readBits<uint8_t>(1))
      .cas_forward_sign(bitstream.getFlag())
      .cas_left_sign(bitstream.getFlag())
      .cas_up_sign(bitstream.getFlag());

  return x;
}

void CoordinateAxisSystemParams::encodeTo(OutputBitstream &bitstream) const {
  bitstream.writeBits(cas_forward_axis(), 2);
  bitstream.writeBits(cas_delta_left_axis_minus1(), 1);
  bitstream.putFlag(cas_forward_sign());
  bitstream.putFlag(cas_left_sign());
  bitstream.putFlag(cas_up_sign());
}

auto operator<<(ostream &stream, const VuiParameters &x) -> ostream & {
  return stream << x.coordinate_axis_system_params();
}

auto VuiParameters::decodeFrom(InputBitstream &bitstream) -> VuiParameters {
  const auto cas = CoordinateAxisSystemParams::decodeFrom(bitstream);
  return VuiParameters{cas};
}

void VuiParameters::encodeTo(OutputBitstream &bitstream) const {
  coordinate_axis_system_params().encodeTo(bitstream);
}
} // namespace TMIV::MivBitstream
