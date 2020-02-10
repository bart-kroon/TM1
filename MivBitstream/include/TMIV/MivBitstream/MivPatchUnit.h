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
#define _TMIV_MIVBITSTREAM_MIVPATCHUNIT_H_

#include <TMIV/Common/Vector.h>
#include <TMIV/MivBitstream/AtlasTileGroupLayerRBSP.h>

#include <type_traits>

namespace TMIV::MivBitstream {
// 23090-12 proposal: miv_patch_unit
//
// Implemented as a view (const or mutable) on PatchDataUnit
template <typename PDU> class MivPatchUnit {
public:
  static_assert(std::is_same_v<PDU, PatchDataUnit> ||
                std::is_same_v<PDU, const PatchDataUnit>);

  explicit MivPatchUnit(PDU &pdu) noexcept;

  // Direct access to the 23090-5 patch_data_unit
  constexpr auto &patch_data_unit() const noexcept;

  // Get the position of the patch in the atlas (left-most column)
  // Specified by reference to 23090-5
  constexpr auto pdu_2d_pos_x() const noexcept;

  // Get the position of the patch in the atlas (top-most row)
  // Specified by reference to 23090-5
  constexpr auto pdu_2d_pos_y() const noexcept;

  // Get the size of the patch in the atlas (width, number of columns)
  // Specified by reference to 23090-5
  constexpr auto pdu_2d_size_x() const noexcept;

  // Get the size of the patch in the atlas (height, number of rows)
  // Specified by reference to 23090-5
  constexpr auto pdu_2d_size_y() const noexcept;

  // Get the position of the patch in the view (left-most column)
  // This syntax element replaces pdu_3d_pos_x
  constexpr auto mpu_view_pos_x() const noexcept;

  // Get the position of the patch in the view (top-most row)
  // This syntax element replaces pdu_3d_pos_y
  constexpr auto mpu_view_pos_y() const noexcept;

  // Get the index into view_params_list
  // This syntax element replaces pdu_projection_id
  constexpr auto mpu_view_id() const noexcept;

  // Get the patch orientation index
  // Specified by reference to 23090-5
  constexpr auto pdu_orientation_index() const noexcept;

  // Set the position of the patch in the atlas (left-most column)
  // Specified by reference to 23090-5
  template <typename = std::enable_if<!std::is_const_v<PDU>>>
  constexpr auto &pdu_2d_pos_x(std::uint32_t value) noexcept;

  // Set the position of the patch in the atlas (top-most row)
  // Specified by reference to 23090-5
  template <typename = std::enable_if<!std::is_const_v<PDU>>>
  constexpr auto &pdu_2d_pos_y(std::uint32_t value) noexcept;

  // Set the size of the patch in the atlas (width, number of columns)
  // Specified by reference to 23090-5
  template <typename = std::enable_if<!std::is_const_v<PDU>>>
  constexpr auto &pdu_2d_size_x(std::uint32_t value) noexcept;

  // Set the size of the patch in the atlas (height, number of rows)
  // Specified by reference to 23090-5
  template <typename = std::enable_if<!std::is_const_v<PDU>>>
  constexpr auto &pdu_2d_size_y(std::uint32_t value) noexcept;

  // Set the position of the patch in the view (left-most column)
  // This syntax element replaces pdu_3d_pos_x
  template <typename = std::enable_if<!std::is_const_v<PDU>>>
  constexpr auto &mpu_view_pos_x(std::uint32_t value) noexcept;

  // Set the position of the patch in the view (top-most row)
  // This syntax element replaces pdu_3d_pos_y
  template <typename = std::enable_if<!std::is_const_v<PDU>>>
  constexpr auto &mpu_view_pos_y(std::uint32_t value) noexcept;

  // Set the index into view_params_list
  // This syntax element replaces pdu_projection_id
  template <typename = std::enable_if<!std::is_const_v<PDU>>>
  constexpr auto &mpu_view_id(std::uint16_t value) noexcept;

  // Set the patch orientation index
  // Specified by reference to 23090-5
  template <typename = std::enable_if<!std::is_const_v<PDU>>>
  constexpr auto &pdu_orientation_index(FlexiblePatchOrientation value) noexcept;

  // Get the size of the patch in the atlas (width, height)
  // This is a convenience function based on pdu_2d_size_{x,y}
  auto patchSizeInAtlas() const -> Common::Vec2i;

  // Get the sof the patch in the atlas (width, height)
  // This is a convenience function based on pdu_2d_size_{x,y} and pdu_orientation_index
  auto patchSizeInView() const -> Common::Vec2i;

  // Get the top-left position of the patch in the view
  // This is a convenience function based on mpd_view_pos_{x,y}
  auto posInView() const -> Common::Vec2i;

  // Get the top-left position of the patch in the atlas
  // This is a convenience function based on pdu_2d_pos_{x,y}
  auto posInAtlas() const -> Common::Vec2i;

  auto printTo(std::ostream &stream, std::size_t patchIdx) const -> std::ostream &;

  template <typename OtherPDU>
  auto operator==(const MivPatchUnit<OtherPDU> &other) const noexcept -> bool;

  template <typename OtherPDU>
  auto operator!=(const MivPatchUnit<OtherPDU> &other) const noexcept -> bool;

private:
  PDU &m_pdu;
};
} // namespace TMIV::MivBitstream

#include "MivPatchUnit.hpp"

#endif
