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

#include <TMIV/MivBitstream/MivPatchUnit.h>

#include "verify.h"

using namespace std;
using namespace TMIV::VpccBitstream;

namespace TMIV::MivBitstream {
template <typename PDU> MivPatchUnit<PDU>::MivPatchUnit(PDU &pdu) noexcept : m_pdu{pdu} {
  VERIFY_MIVBITSTREAM(!pdu.pdu_3d_pos_delta_max_z());
  VERIFY_MIVBITSTREAM(!pdu.pdu_lod());
}

template <typename PDU> auto MivPatchUnit<PDU>::patchSizeInAtlas() const -> Common::Vec2i {
  return {int(pdu_2d_size_x()), int(pdu_2d_size_y())};
}

template <typename PDU> auto MivPatchUnit<PDU>::patchSizeInView() const -> Common::Vec2i {
  using VpccBitstream::FlexiblePatchOrientation;

  switch (pdu_orientation_index()) {
  case FlexiblePatchOrientation::FPO_NULL:
  case FlexiblePatchOrientation::FPO_MIRROR:
  case FlexiblePatchOrientation::FPO_MROT180:
  case FlexiblePatchOrientation::FPO_ROT180:
    return {int(mpu_view_pos_x()), int(mpu_view_pos_y())};
  case FlexiblePatchOrientation::FPO_MROT90:
  case FlexiblePatchOrientation::FPO_ROT270:
  case FlexiblePatchOrientation::FPO_ROT90:
  case FlexiblePatchOrientation::FPO_SWAP:
    return {int(mpu_view_pos_y()), int(mpu_view_pos_x())};
  default:
    MIVBITSTREAM_ERROR("Unknown orientation index");
  }
}

template <typename PDU> auto MivPatchUnit<PDU>::posInView() const -> Common::Vec2i {
  return {int(mpu_view_pos_x()), int(mpu_view_pos_y())};
}

template <typename PDU> auto MivPatchUnit<PDU>::posInAtlas() const -> Common::Vec2i {
  return {int(pdu_2d_pos_x()), int(pdu_2d_pos_y())};
}

template <typename PDU>
auto MivPatchUnit<PDU>::printTo(std::ostream &stream, std::size_t patchIdx) const
    -> std::ostream & {
  return stream << "pdu_2d_pos_x( " << patchIdx << " )=" << pdu_2d_pos_x() << "\npdu_2d_pos_y( "
                << patchIdx << " )=" << pdu_2d_pos_y() << "\npdu_2d_size_x( " << patchIdx
                << " )=" << pdu_2d_size_x() << "\npdu_2d_size_y( " << patchIdx
                << " )=" << pdu_2d_size_y() << "\nmpu_view_pos_x( " << patchIdx
                << " )=" << mpu_view_pos_x() << "\nmpu_view_pos_y( " << patchIdx
                << " )=" << mpu_view_pos_y() << "\nmpu_view_id( " << patchIdx
                << " )=" << mpu_view_id() << "\npdu_orientation_index( " << patchIdx
                << " )=" << pdu_orientation_index() << '\n';
}

template class MivPatchUnit<VpccBitstream::PatchDataUnit>;
template class MivPatchUnit<const VpccBitstream::PatchDataUnit>;
} // namespace TMIV::MivBitstream