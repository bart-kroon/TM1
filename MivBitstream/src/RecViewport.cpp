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

#include <TMIV/MivBitstream/RecViewport.h>

#include "verify.h"

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto RecViewportParams::operator==(const RecViewportParams &other) const noexcept -> bool {
  if (this->rec_viewport_id == other.rec_viewport_id &&
      this->rec_viewport_cancel_flag == other.rec_viewport_cancel_flag &&
      this->rec_viewport_persistence_flag == other.rec_viewport_persistence_flag &&
      this->rec_viewport_center_view_flag == other.rec_viewport_center_view_flag &&
      this->rec_viewport_left_view_flag == other.rec_viewport_left_view_flag &&
      this->rec_viewport_pos_x == other.rec_viewport_pos_x &&
      this->rec_viewport_pos_y == other.rec_viewport_pos_y &&
      this->rec_viewport_pos_z == other.rec_viewport_pos_z &&
      this->rec_viewport_quat_x == other.rec_viewport_quat_x &&
      this->rec_viewport_quat_y == other.rec_viewport_quat_y &&
      this->rec_viewport_quat_z == other.rec_viewport_quat_z &&
      this->rec_viewport_hor_range == other.rec_viewport_hor_range &&
      this->rec_viewport_ver_range == other.rec_viewport_ver_range)
    return true;
  else
    return false;
}

auto RecViewportParams::operator!=(const RecViewportParams &other) const noexcept -> bool {
  return !operator==(other);
}

RecViewport::RecViewport(RecViewportParams value) : m_RecViewportParams{move(value)} {}

auto operator<<(ostream &stream, const RecViewport &y) -> ostream & {
  RecViewportParams x = y.m_RecViewportParams;
  uint16_t seiId = x.rec_viewport_id;
  stream << "rec_viewport_id=" << x.rec_viewport_id << '\n';
  stream << "rec_viewport_cancel_flag( " << seiId << " )=" << x.rec_viewport_cancel_flag << '\n';
  if (!x.rec_viewport_cancel_flag) {
    if (x.rec_viewport_persistence_flag.has_value())
      stream << "rec_viewport_persistence_flag( " << seiId
             << " )=" << *x.rec_viewport_persistence_flag << '\n';
    if (x.rec_viewport_center_view_flag.has_value()) {
      stream << "rec_viewport_center_view_flag( " << seiId
             << " )=" << *x.rec_viewport_center_view_flag << '\n';
      if ((!*x.rec_viewport_center_view_flag) && x.rec_viewport_left_view_flag.has_value())
        stream << "rec_viewport_left_view_flag( " << seiId
               << " )=" << *x.rec_viewport_left_view_flag << '\n';
    }
    if (x.rec_viewport_pos_x.has_value())
      stream << "rec_viewport_pos_x( " << seiId << " )=" << *x.rec_viewport_pos_x << '\n';
    if (x.rec_viewport_pos_y.has_value())
      stream << "rec_viewport_pos_y( " << seiId << " )=" << *x.rec_viewport_pos_y << '\n';
    if (x.rec_viewport_pos_z.has_value())
      stream << "rec_viewport_pos_z( " << seiId << " )=" << *x.rec_viewport_pos_z << '\n';
    if (x.rec_viewport_quat_x.has_value())
      stream << "rec_viewport_quat_x( " << seiId << " )=" << *x.rec_viewport_quat_x << '\n';
    if (x.rec_viewport_quat_y.has_value())
      stream << "rec_viewport_quat_y( " << seiId << " )=" << *x.rec_viewport_quat_y << '\n';
    if (x.rec_viewport_quat_z.has_value())
      stream << "rec_viewport_quat_z( " << seiId << " )=" << *x.rec_viewport_quat_z << '\n';
    if (x.rec_viewport_hor_range.has_value())
      stream << "rec_viewport_hor_range( " << seiId << " )=" << *x.rec_viewport_hor_range << '\n';
    if (x.rec_viewport_ver_range.has_value())
      stream << "rec_viewport_ver_range( " << seiId << " )=" << *x.rec_viewport_ver_range << '\n';
  }
  return stream;
}

auto RecViewport::operator==(const RecViewport &other) const noexcept -> bool {
  return m_RecViewportParams == other.m_RecViewportParams;
}

auto RecViewport::operator!=(const RecViewport &other) const noexcept -> bool {
  return !operator==(other);
}

auto RecViewport::decodeFrom(InputBitstream &bitstream) -> RecViewport {
  RecViewportParams x = RecViewportParams();
  x.rec_viewport_id = bitstream.readBits<uint16_t>(10);
  x.rec_viewport_cancel_flag = bitstream.getFlag();
  if (!x.rec_viewport_cancel_flag) {
    x.rec_viewport_persistence_flag = bitstream.getFlag();
    x.rec_viewport_center_view_flag = bitstream.getFlag();
    if (!*x.rec_viewport_center_view_flag)
      x.rec_viewport_left_view_flag = bitstream.getFlag();
    x.rec_viewport_pos_x = bitstream.getFloat32();
    x.rec_viewport_pos_y = bitstream.getFloat32();
    x.rec_viewport_pos_z = bitstream.getFloat32();
    x.rec_viewport_quat_x = bitstream.getFloat32();
    x.rec_viewport_quat_y = bitstream.getFloat32();
    x.rec_viewport_quat_z = bitstream.getFloat32();
    x.rec_viewport_hor_range = bitstream.getFloat32();
    x.rec_viewport_ver_range = bitstream.getFloat32();
  }
  RecViewport y = RecViewport();
  y.m_RecViewportParams = x;
  return y;
}

void RecViewport::encodeTo(OutputBitstream &bitstream) const {
  bitstream.writeBits(m_RecViewportParams.rec_viewport_id, 10);
  bitstream.putFlag(m_RecViewportParams.rec_viewport_cancel_flag);
  if (!m_RecViewportParams.rec_viewport_cancel_flag) {
    if (m_RecViewportParams.rec_viewport_persistence_flag.has_value() &&
        m_RecViewportParams.rec_viewport_center_view_flag.has_value() &&
        m_RecViewportParams.rec_viewport_pos_x.has_value() &&
        m_RecViewportParams.rec_viewport_pos_y.has_value() &&
        m_RecViewportParams.rec_viewport_pos_z.has_value() &&
        m_RecViewportParams.rec_viewport_quat_x.has_value() &&
        m_RecViewportParams.rec_viewport_quat_y.has_value() &&
        m_RecViewportParams.rec_viewport_quat_z.has_value() &&
        m_RecViewportParams.rec_viewport_hor_range.has_value() &&
        m_RecViewportParams.rec_viewport_ver_range.has_value()) {
      bitstream.putFlag(*m_RecViewportParams.rec_viewport_persistence_flag);
      bitstream.putFlag(*m_RecViewportParams.rec_viewport_center_view_flag);
      if (!*m_RecViewportParams.rec_viewport_center_view_flag) {
        if (m_RecViewportParams.rec_viewport_left_view_flag.has_value())
          bitstream.putFlag(*m_RecViewportParams.rec_viewport_left_view_flag);
      }
      bitstream.putFloat32(*m_RecViewportParams.rec_viewport_pos_x);
      bitstream.putFloat32(*m_RecViewportParams.rec_viewport_pos_y);
      bitstream.putFloat32(*m_RecViewportParams.rec_viewport_pos_z);
      bitstream.putFloat32(*m_RecViewportParams.rec_viewport_quat_x);
      bitstream.putFloat32(*m_RecViewportParams.rec_viewport_quat_y);
      bitstream.putFloat32(*m_RecViewportParams.rec_viewport_quat_z);
      bitstream.putFloat32(*m_RecViewportParams.rec_viewport_hor_range);
      bitstream.putFloat32(*m_RecViewportParams.rec_viewport_ver_range);
    }
  }
}
} // namespace TMIV::MivBitstream