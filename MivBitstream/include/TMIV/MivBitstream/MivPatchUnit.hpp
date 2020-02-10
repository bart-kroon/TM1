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

#ifndef _TMIV_MIVBITSTREAM_MIVPATCHUNIT_H_
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
template <typename PDU> constexpr auto &MivPatchUnit<PDU>::patch_data_unit() const noexcept {
  return m_pdu;
}

template <typename PDU> constexpr auto MivPatchUnit<PDU>::pdu_2d_pos_x() const noexcept {
  return m_pdu.pdu_2d_pos_x();
}

template <typename PDU> constexpr auto MivPatchUnit<PDU>::pdu_2d_pos_y() const noexcept {
  return m_pdu.pdu_2d_pos_y();
}

template <typename PDU> constexpr auto MivPatchUnit<PDU>::pdu_2d_size_x() const noexcept {
  return m_pdu.pdu_2d_size_x();
}

template <typename PDU> constexpr auto MivPatchUnit<PDU>::pdu_2d_size_y() const noexcept {
  return m_pdu.pdu_2d_size_y();
}

template <typename PDU> constexpr auto MivPatchUnit<PDU>::mpu_view_pos_x() const noexcept {
  return m_pdu.pdu_3d_pos_x();
}

template <typename PDU> constexpr auto MivPatchUnit<PDU>::mpu_view_pos_y() const noexcept {
  return m_pdu.pdu_3d_pos_y();
}

template <typename PDU>

constexpr auto MivPatchUnit<PDU>::mpu_view_id() const noexcept {
  return m_pdu.pdu_projection_id();
}

template <typename PDU> constexpr auto MivPatchUnit<PDU>::pdu_orientation_index() const noexcept {
  return m_pdu.pdu_orientation_index();
}

template <typename PDU>
template <typename>
constexpr auto &MivPatchUnit<PDU>::pdu_2d_pos_x(std::uint32_t value) noexcept {
  m_pdu.pdu_2d_pos_x(value);
  return *this;
}

template <typename PDU>
template <typename>
constexpr auto &MivPatchUnit<PDU>::pdu_2d_pos_y(std::uint32_t value) noexcept {
  m_pdu.pdu_2d_pos_y(value);
  return *this;
}

template <typename PDU>
template <typename>
constexpr auto &MivPatchUnit<PDU>::pdu_2d_size_x(std::uint32_t value) noexcept {
  m_pdu.pdu_2d_size_x(value);
  return *this;
}

template <typename PDU>
template <typename>
constexpr auto &MivPatchUnit<PDU>::pdu_2d_size_y(std::uint32_t value) noexcept {
  m_pdu.pdu_2d_size_y(value);
  return *this;
}

template <typename PDU>
template <typename>
constexpr auto &MivPatchUnit<PDU>::mpu_view_pos_x(std::uint32_t value) noexcept {
  m_pdu.pdu_3d_pos_x(value);
  return *this;
}

template <typename PDU>
template <typename>
constexpr auto &MivPatchUnit<PDU>::mpu_view_pos_y(std::uint32_t value) noexcept {
  m_pdu.pdu_3d_pos_y(value);
  return *this;
}

template <typename PDU>
template <typename>
constexpr auto &MivPatchUnit<PDU>::mpu_view_id(std::uint16_t value) noexcept {
  m_pdu.pdu_projection_id(value);
  return *this;
}

template <typename PDU>
template <typename>
constexpr auto &MivPatchUnit<PDU>::pdu_orientation_index(FlexiblePatchOrientation value) noexcept {
  m_pdu.pdu_orientation_index(value);
  return *this;
}

template <typename PDU>
template <typename OtherPDU>
auto MivPatchUnit<PDU>::operator==(const MivPatchUnit<OtherPDU> &other) const noexcept -> bool {
  return patch_data_unit() == other.patch_data_unit();
}

template <typename PDU>
template <typename OtherPDU>
auto MivPatchUnit<PDU>::operator!=(const MivPatchUnit<OtherPDU> &other) const noexcept -> bool {
  return !operator==(other);
}
} // namespace TMIV::MivBitstream
