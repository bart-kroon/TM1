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

#ifndef TMIV_MIVBITSTREAM_VIEWPORTCAMERAPARAMETERS_H
#define TMIV_MIVBITSTREAM_VIEWPORTCAMERAPARAMETERS_H

#include <TMIV/MivBitstream/ViewParamsList.h>

namespace TMIV::MivBitstream {
// 23090-5: viewport_camera_parameters( )
struct ViewportCameraParameters {
  uint16_t vcp_camera_id{};
  bool vcp_cancel_flag{};
  bool vcp_persistent_flag{};
  CiCamType vcp_camera_type{};
  uint32_t vcp_erp_horizontal_fov{};
  uint32_t vcp_erp_vertical_fov{};
  float vcp_perspective_aspect_ratio{};
  uint32_t vcp_perspective_horizontal_fov{};
  float vcp_ortho_aspect_ratio{};
  float vcp_ortho_horizontal_size{};
  float vcp_clipping_near_plane{};
  float vcp_clipping_far_plane{};

  [[nodiscard]] auto vcp_erp_horizontal_fov_in_degrees() const -> float;
  [[nodiscard]] auto vcp_erp_vertical_fov_in_degrees() const -> float;
  [[nodiscard]] auto vcp_perspective_horizontal_fov_in_degrees() const -> float;
  [[nodiscard]] auto vcp_perspective_vertical_fov_in_degrees() const -> float;

  friend auto operator<<(std::ostream &stream, const ViewportCameraParameters &vcp)
      -> std::ostream &;
  auto operator==(const ViewportCameraParameters &other) const -> bool;
  auto operator!=(const ViewportCameraParameters &other) const -> bool {
    return !operator==(other);
  }

  static auto decodeFrom(Common::InputBitstream &stream) -> ViewportCameraParameters;
  void encodeTo(Common::OutputBitstream &stream) const;

  static auto fromViewParams(const ViewParams &viewParams) -> ViewportCameraParameters;
};

} // namespace TMIV::MivBitstream

#endif
