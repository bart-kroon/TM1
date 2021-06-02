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

#include <TMIV/MivBitstream/ViewportCameraParameters.h>

namespace TMIV::MivBitstream {
constexpr auto unit2deg(uint32_t v) -> float { return static_cast<float>(v) / 65536.F; }
auto deg2unit(float v, uint32_t delta) -> uint32_t {
  return std::clamp(static_cast<uint32_t>(std::round(v * 65536.F)), 0U, delta * 65536U - 1U);
}

auto ViewportCameraParameters::vcp_erp_horizontal_fov_in_degrees() const -> float {
  return unit2deg(vcp_erp_horizontal_fov);
}

auto ViewportCameraParameters::vcp_erp_vertical_fov_in_degrees() const -> float {
  return unit2deg(vcp_erp_vertical_fov);
}

auto ViewportCameraParameters::vcp_perspective_horizontal_fov_in_degrees() const -> float {
  return unit2deg(vcp_perspective_horizontal_fov);
}

auto ViewportCameraParameters::vcp_perspective_vertical_fov_in_degrees() const -> float {
  return Common::rad2deg(
      2.F *
      std::atan(std::tan(0.5F * Common::deg2rad(vcp_perspective_horizontal_fov_in_degrees())) /
                vcp_perspective_aspect_ratio));
}

auto operator<<(std::ostream &stream, const ViewportCameraParameters &vcp) -> std::ostream & {
  stream << "vcp_camera_id=" << vcp.vcp_camera_id << '\n';
  stream << "vcp_cancel_flag=" << std::boolalpha << vcp.vcp_cancel_flag << '\n';

  if (vcp.vcp_camera_id > 0 && !vcp.vcp_cancel_flag) {
    stream << "vcp_persistent_flag=" << std::boolalpha << vcp.vcp_persistent_flag << '\n';
    stream << "vcp_camera_type=" << vcp.vcp_camera_type << "\n";

    switch (vcp.vcp_camera_type) {
    case CiCamType::equirectangular:
      stream << "vcp_erp_horizontal_fov=" << vcp.vcp_erp_horizontal_fov << '\n';
      stream << "vcp_erp_vertical_fov=" << vcp.vcp_erp_vertical_fov << '\n';
      break;
    case CiCamType::perspective:
      stream << "vcp_perspective_aspect_ratio=" << vcp.vcp_perspective_aspect_ratio << '\n';
      stream << "vcp_perspective_horizontal_fov=" << vcp.vcp_perspective_horizontal_fov << '\n';
      break;
    case CiCamType::orthographic:
      stream << "vcp_ortho_aspect_ratio=" << vcp.vcp_ortho_aspect_ratio << '\n';
      stream << "vcp_ortho_horizontal_size=" << vcp.vcp_ortho_horizontal_size << '\n';
      break;
    }

    stream << "vcp_clipping_near_plane=" << vcp.vcp_clipping_near_plane << '\n';
    stream << "vcp_clipping_far_plane=" << vcp.vcp_clipping_far_plane << '\n';
  }

  return stream;
}

auto ViewportCameraParameters::operator==(const ViewportCameraParameters &other) const -> bool {
  return (vcp_camera_id == other.vcp_camera_id) && (vcp_cancel_flag == other.vcp_cancel_flag) &&
         (vcp_persistent_flag == other.vcp_persistent_flag) &&
         (vcp_camera_type == other.vcp_camera_type) &&
         (vcp_erp_horizontal_fov == other.vcp_erp_horizontal_fov) &&
         (vcp_erp_vertical_fov == other.vcp_erp_vertical_fov) &&
         (vcp_perspective_aspect_ratio == other.vcp_perspective_aspect_ratio) &&
         (vcp_perspective_horizontal_fov == other.vcp_perspective_horizontal_fov) &&
         (vcp_ortho_aspect_ratio == other.vcp_ortho_aspect_ratio) &&
         (vcp_ortho_horizontal_size == other.vcp_ortho_horizontal_size) &&
         (vcp_clipping_near_plane == other.vcp_clipping_near_plane) &&
         (vcp_clipping_far_plane == other.vcp_clipping_far_plane);
}

auto ViewportCameraParameters::decodeFrom(Common::InputBitstream &stream)
    -> ViewportCameraParameters {
  ViewportCameraParameters vcp;

  vcp.vcp_camera_id = stream.readBits<uint16_t>(10);
  vcp.vcp_cancel_flag = stream.getFlag();

  if (vcp.vcp_camera_id > 0 && !vcp.vcp_cancel_flag) {
    vcp.vcp_persistent_flag = stream.getFlag();
    vcp.vcp_camera_type = static_cast<CiCamType>(stream.readBits<uint8_t>(3));

    switch (vcp.vcp_camera_type) {
    case CiCamType::equirectangular:
      vcp.vcp_erp_horizontal_fov = stream.getUint32();
      vcp.vcp_erp_vertical_fov = stream.getUint32();
      break;
    case CiCamType::perspective:
      vcp.vcp_perspective_aspect_ratio = stream.getFloat32();
      vcp.vcp_perspective_horizontal_fov = stream.getUint32();
      break;
    case CiCamType::orthographic:
      vcp.vcp_ortho_aspect_ratio = stream.getFloat32();
      vcp.vcp_ortho_horizontal_size = stream.getFloat32();
      break;
    }

    vcp.vcp_clipping_near_plane = stream.getFloat32();
    vcp.vcp_clipping_far_plane = stream.getFloat32();
  }

  return vcp;
}

void ViewportCameraParameters::encodeTo(Common::OutputBitstream &stream) const {
  stream.writeBits<uint16_t>(vcp_camera_id, 10);
  stream.putFlag(vcp_cancel_flag);

  if (vcp_camera_id > 0 && !vcp_cancel_flag) {
    stream.putFlag(vcp_persistent_flag);
    stream.writeBits<uint8_t>(static_cast<uint8_t>(vcp_camera_type), 3);

    switch (vcp_camera_type) {
    case CiCamType::equirectangular:
      stream.putUint32(vcp_erp_horizontal_fov);
      stream.putUint32(vcp_erp_vertical_fov);
      break;
    case CiCamType::perspective:
      stream.putFloat32(vcp_perspective_aspect_ratio);
      stream.putUint32(vcp_perspective_horizontal_fov);
      break;
    case CiCamType::orthographic:
      stream.putFloat32(vcp_ortho_aspect_ratio);
      stream.putFloat32(vcp_ortho_horizontal_size);
      break;
    }

    stream.putFloat32(vcp_clipping_near_plane);
    stream.putFloat32(vcp_clipping_far_plane);
  }
}

auto ViewportCameraParameters::fromViewParams(const ViewParams &viewParams)
    -> ViewportCameraParameters {
  ViewportCameraParameters vcp;
  auto w = viewParams.ci.ci_projection_plane_width_minus1() + 1;
  auto h = viewParams.ci.ci_projection_plane_height_minus1() + 1;

  vcp.vcp_camera_id = 1;
  vcp.vcp_cancel_flag = false;
  vcp.vcp_persistent_flag = true;
  vcp.vcp_camera_type = viewParams.ci.ci_cam_type();

  switch (vcp.vcp_camera_type) {
  case CiCamType::equirectangular:
    vcp.vcp_erp_horizontal_fov =
        deg2unit(viewParams.ci.ci_erp_phi_max() - viewParams.ci.ci_erp_phi_min(), 360U);
    vcp.vcp_erp_vertical_fov =
        deg2unit(viewParams.ci.ci_erp_theta_max() - viewParams.ci.ci_erp_theta_min(), 180U);
    break;
  case CiCamType::perspective:
    vcp.vcp_perspective_aspect_ratio = static_cast<float>(w) / static_cast<float>(h);
    vcp.vcp_perspective_horizontal_fov =
        deg2unit(Common::rad2deg(2.F * std::atan(static_cast<float>(w) /
                                                 (2.F * viewParams.ci.ci_perspective_focal_hor()))),
                 180U);
    break;
  case CiCamType::orthographic:
    vcp.vcp_ortho_aspect_ratio = static_cast<float>(w) / static_cast<float>(h);
    vcp.vcp_ortho_horizontal_size = static_cast<float>(h);
    break;
  }

  vcp.vcp_clipping_near_plane = 1.F / viewParams.dq.dq_norm_disp_high();
  vcp.vcp_clipping_far_plane = 1.F / viewParams.dq.dq_norm_disp_low();

  return vcp;
}

} // namespace TMIV::MivBitstream
