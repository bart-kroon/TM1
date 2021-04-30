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

#ifndef TMIV_MIVBITSTREAM_VIEWPORTPOSITION_H
#define TMIV_MIVBITSTREAM_VIEWPORTPOSITION_H

#include <TMIV/MivBitstream/ViewParamsList.h>

namespace TMIV::MivBitstream {
// 23090-5: viewport_position( )
struct ViewportPosition {
  uint16_t vp_viewport_id{};
  bool vp_camera_parameters_present_flag{};
  uint16_t vp_vcp_camera_id{};
  bool vp_cancel_flag{};
  bool vp_persistent_flag{};
  Common::Vec3f vp_position{};
  int16_t vp_rotation_qx{};
  int16_t vp_rotation_qy{};
  int16_t vp_rotation_qz{};
  bool vp_center_view_flag{};
  bool vp_left_view_flag{};

  [[nodiscard]] auto vp_orientation() const -> Common::QuatF;

  friend auto operator<<(std::ostream &stream, const ViewportPosition &vp) -> std::ostream &;
  auto operator==(const ViewportPosition &other) const -> bool;
  auto operator!=(const ViewportPosition &other) const -> bool { return !operator==(other); }

  static auto decodeFrom(Common::InputBitstream &stream) -> ViewportPosition;
  void encodeTo(Common::OutputBitstream &stream) const;

  static auto fromViewParams(const ViewParams &viewParams) -> ViewportPosition;
};

} // namespace TMIV::MivBitstream

#endif