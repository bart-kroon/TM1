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

#include <TMIV/MivBitstream/RecommendedViewport.h>

#include "verify.h"

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto operator<<(ostream &stream, const RecommendedViewport &x) -> ostream & {
  stream << "rec_viewport_id=" << x.m_rec_viewport_id << '\n';
  stream << "rec_viewport_cancel_flag=" << x.m_rec_viewport_cancel_flag << '\n';
  if (!x.m_rec_viewport_cancel_flag) {
    stream << "rec_viewport_persistence_flag=" << x.m_rec_viewport_persistence_flag << '\n';
    stream << "rec_viewport_center_view_flag=" << x.m_rec_viewport_center_view_flag << '\n';
    if (!x.m_rec_viewport_center_view_flag)
      stream << "rec_viewport_left_view_flag=" << x.m_rec_viewport_left_view_flag << '\n';
    stream << "rec_viewport_pos_x=" << x.m_rec_viewport_pos_x << '\n';
    stream << "rec_viewport_pos_y=" << x.m_rec_viewport_pos_y << '\n';
    stream << "rec_viewport_pos_z=" << x.m_rec_viewport_pos_z << '\n';
    stream << "rec_viewport_quat_x=" << x.m_rec_viewport_quat_x << '\n';
    stream << "rec_viewport_quat_y=" << x.m_rec_viewport_quat_y << '\n';
    stream << "rec_viewport_quat_z=" << x.m_rec_viewport_quat_z << '\n';
    stream << "rec_viewport_hor_range=" << x.m_rec_viewport_hor_range << '\n';
    stream << "rec_viewport_ver_range=" << x.m_rec_viewport_ver_range << '\n';
  }
  return stream;
}

auto RecommendedViewport::operator==(const RecommendedViewport &other) const noexcept -> bool {
  if (this->m_rec_viewport_id == other.m_rec_viewport_id &&
      this->m_rec_viewport_cancel_flag == other.m_rec_viewport_cancel_flag &&
      this->m_rec_viewport_persistence_flag == other.m_rec_viewport_persistence_flag &&
      this->m_rec_viewport_center_view_flag == other.m_rec_viewport_center_view_flag &&
      this->m_rec_viewport_left_view_flag == other.m_rec_viewport_left_view_flag &&
      this->m_rec_viewport_pos_x == other.m_rec_viewport_pos_x &&
      this->m_rec_viewport_pos_y == other.m_rec_viewport_pos_y &&
      this->m_rec_viewport_pos_z == other.m_rec_viewport_pos_z &&
      this->m_rec_viewport_quat_x == other.m_rec_viewport_quat_x &&
      this->m_rec_viewport_quat_y == other.m_rec_viewport_quat_y &&
      this->m_rec_viewport_quat_z == other.m_rec_viewport_quat_z &&
      this->m_rec_viewport_hor_range == other.m_rec_viewport_hor_range &&
      this->m_rec_viewport_ver_range == other.m_rec_viewport_ver_range)
    return true;
  else
    return false;
}

auto RecommendedViewport::operator!=(const RecommendedViewport &other) const noexcept -> bool {
  return !operator==(other);
}

auto RecommendedViewport::rec_viewport_sei_size() const noexcept -> size_t {
  size_t SeiSize = sizeof(this->m_rec_viewport_id) + sizeof(this->m_rec_viewport_cancel_flag) +
                   sizeof(this->m_rec_viewport_persistence_flag) +
                   sizeof(this->m_rec_viewport_center_view_flag) +
                   sizeof(this->m_rec_viewport_left_view_flag) +
                   sizeof(this->m_rec_viewport_pos_x) + sizeof(this->m_rec_viewport_pos_y) +
                   sizeof(this->m_rec_viewport_pos_z) + sizeof(this->m_rec_viewport_quat_x) +
                   sizeof(this->m_rec_viewport_quat_y) + sizeof(this->m_rec_viewport_quat_z) +
                   sizeof(this->m_rec_viewport_hor_range) + sizeof(m_rec_viewport_ver_range);
  return SeiSize;
}

auto RecommendedViewport::decodeFrom(InputBitstream &bitstream) -> RecommendedViewport {
  RecommendedViewport x = RecommendedViewport();
  x.m_rec_viewport_id = bitstream.readBits<uint16_t>(10);
  x.m_rec_viewport_cancel_flag = bitstream.getFlag();
  if (!x.m_rec_viewport_cancel_flag) {
    x.m_rec_viewport_persistence_flag = bitstream.getFlag();
    x.m_rec_viewport_center_view_flag = bitstream.getFlag();
    if (!x.m_rec_viewport_center_view_flag)
      x.m_rec_viewport_left_view_flag = bitstream.getFlag();
    x.m_rec_viewport_pos_x = bitstream.getFloat32();
    x.m_rec_viewport_pos_y = bitstream.getFloat32();
    x.m_rec_viewport_pos_z = bitstream.getFloat32();
    x.m_rec_viewport_quat_x = bitstream.getFloat32();
    x.m_rec_viewport_quat_y = bitstream.getFloat32();
    x.m_rec_viewport_quat_z = bitstream.getFloat32();
    x.m_rec_viewport_hor_range = bitstream.getFloat32();
    x.m_rec_viewport_ver_range = bitstream.getFloat32();
  }
  return x;
}

void RecommendedViewport::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putUExpGolomb(rec_viewport_sei_size());
  bitstream.writeBits(m_rec_viewport_id, 10);
  bitstream.putFlag(m_rec_viewport_cancel_flag);
  if (!m_rec_viewport_cancel_flag) {
    bitstream.putFlag(m_rec_viewport_persistence_flag);
    bitstream.putFlag(m_rec_viewport_center_view_flag);
    if (!m_rec_viewport_center_view_flag)
      bitstream.putFlag(m_rec_viewport_left_view_flag);
    bitstream.putFloat32(m_rec_viewport_pos_x);
    bitstream.putFloat32(m_rec_viewport_pos_y);
    bitstream.putFloat32(m_rec_viewport_pos_z);
    bitstream.putFloat32(m_rec_viewport_quat_x);
    bitstream.putFloat32(m_rec_viewport_quat_y);
    bitstream.putFloat32(m_rec_viewport_quat_z);
    bitstream.putFloat32(m_rec_viewport_hor_range);
    bitstream.putFloat32(m_rec_viewport_ver_range);
  }
}
} // namespace TMIV::MivBitstream