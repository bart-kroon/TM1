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

#include <TMIV/MivBitstream/FrameOrderCountRBSP.h>

#include <ostream>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto operator<<(ostream &stream, const FrameOrderCountRBSP &x) -> ostream & {
  stream << "frm_order_cnt_lsb=" << x.frm_order_cnt_lsb() << '\n';
  return stream;
}

auto FrameOrderCountRBSP::decodeFrom(std::istream &stream,
                                     const AtlasAdaptationParameterSetRBSP &aaps)
    -> FrameOrderCountRBSP {
  auto x = FrameOrderCountRBSP{};
  InputBitstream bitstream{stream};

  x.frm_order_cnt_lsb(
      bitstream.readBits<uint16_t>(aaps.aaps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4));
  bitstream.rbspTrailingBits();

  return x;
}

auto FrameOrderCountRBSP::decodeFrom(std::istream &stream,
                                     const AtlasSequenceParameterSetRBSP &asps)
    -> FrameOrderCountRBSP {
  auto x = FrameOrderCountRBSP{};
  InputBitstream bitstream{stream};

  x.frm_order_cnt_lsb(
      bitstream.readBits<uint16_t>(asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4));
  bitstream.rbspTrailingBits();

  return x;
}

void FrameOrderCountRBSP::encodeTo(std::ostream &stream,
                                   const AtlasAdaptationParameterSetRBSP &aaps) const {
  OutputBitstream bitstream{stream};

  bitstream.writeBits(frm_order_cnt_lsb(),
                      aaps.aaps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4);
  bitstream.rbspTrailingBits();
}

void FrameOrderCountRBSP::encodeTo(std::ostream &stream,
                                   const AtlasSequenceParameterSetRBSP &asps) const {
  OutputBitstream bitstream{stream};

  bitstream.writeBits(frm_order_cnt_lsb(),
                      asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4);
  bitstream.rbspTrailingBits();
}
} // namespace TMIV::MivBitstream
