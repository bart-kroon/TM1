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

#include <TMIV/MivBitstream/ViewportPosition.h>

namespace TMIV::MivBitstream {

template <typename T> constexpr auto quat2unit(T v) -> int16_t {
  return std::clamp(static_cast<int16_t>(std::round(v * static_cast<T>(16384))),
                    static_cast<int16_t>(-16384), static_cast<int16_t>(16384));
}

template <typename T> constexpr auto unit2quat(int16_t v) -> T {
  return static_cast<T>(v) / static_cast<T>(16384);
}

auto ViewportPosition::vp_orientation() const -> Common::QuatF {
  Common::QuatF q;

  q.x() = unit2quat<float>(vp_rotation_qx);
  q.y() = unit2quat<float>(vp_rotation_qy);
  q.z() = unit2quat<float>(vp_rotation_qz);
  q.w() = std::sqrt(1.F - (q.x() * q.x() + q.y() * q.y() + q.z() * q.z()));

  return q;
}

auto operator<<(std::ostream &stream, const ViewportPosition &vp) -> std::ostream & {
  stream << "vp_viewport_id=" << vp.vp_viewport_id << '\n';
  stream << "vp_camera_parameters_present_flag=" << std::boolalpha
         << vp.vp_camera_parameters_present_flag << '\n';
  if (vp.vp_camera_parameters_present_flag) {
    stream << "vp_vcp_camera_id=" << std::boolalpha << vp.vp_vcp_camera_id << '\n';
  }
  stream << "vp_cancel_flag=" << std::boolalpha << vp.vp_cancel_flag << '\n';
  if (!vp.vp_cancel_flag) {
    stream << "vp_persistent_flag=" << vp.vp_persistent_flag << '\n';
    stream << "vp_position=" << vp.vp_position << '\n';
    stream << "vp_rotation_qx=" << vp.vp_rotation_qx << '\n';
    stream << "vp_rotation_qy=" << vp.vp_rotation_qy << '\n';
    stream << "vp_rotation_qz=" << vp.vp_rotation_qz << '\n';
    stream << "vp_center_view_flag=" << std::boolalpha << vp.vp_center_view_flag << '\n';
    if (!vp.vp_center_view_flag) {
      stream << "vp_left_view_flag=" << std::boolalpha << vp.vp_left_view_flag << '\n';
    }
  }

  return stream;
}

auto ViewportPosition::operator==(const ViewportPosition &other) const -> bool {
  return (vp_viewport_id == other.vp_viewport_id) &&
         (vp_camera_parameters_present_flag == other.vp_camera_parameters_present_flag) &&
         (vp_vcp_camera_id == other.vp_vcp_camera_id) && (vp_cancel_flag == other.vp_cancel_flag) &&
         (vp_persistent_flag == other.vp_persistent_flag) && (vp_position == other.vp_position) &&
         (vp_rotation_qx == other.vp_rotation_qx) && (vp_rotation_qy == other.vp_rotation_qy) &&
         (vp_rotation_qz == other.vp_rotation_qz) &&
         (vp_center_view_flag == other.vp_center_view_flag) &&
         (vp_left_view_flag == other.vp_left_view_flag);
}

auto ViewportPosition::decodeFrom(Common::InputBitstream &stream) -> ViewportPosition {
  ViewportPosition vp;

  vp.vp_viewport_id = stream.getUExpGolomb<uint16_t>();
  vp.vp_camera_parameters_present_flag = stream.getFlag();

  if (vp.vp_camera_parameters_present_flag) {
    vp.vp_vcp_camera_id = stream.readBits<uint16_t>(10);
  }

  vp.vp_cancel_flag = stream.getFlag();

  if (!vp.vp_cancel_flag) {
    vp.vp_persistent_flag = stream.getFlag();

    for (auto d = 0; d < 3; ++d) {
      vp.vp_position[d] = stream.getFloat32();
    }

    vp.vp_rotation_qx = stream.getInt16();
    vp.vp_rotation_qy = stream.getInt16();
    vp.vp_rotation_qz = stream.getInt16();
    vp.vp_center_view_flag = stream.getFlag();

    if (!vp.vp_center_view_flag) {
      vp.vp_left_view_flag = stream.getFlag();
    }
  }

  return vp;
}

void ViewportPosition::encodeTo(Common::OutputBitstream &stream) const {

  stream.putUExpGolomb<uint16_t>(vp_viewport_id);
  stream.putFlag(vp_camera_parameters_present_flag);

  if (vp_camera_parameters_present_flag) {
    stream.writeBits<uint16_t>(vp_vcp_camera_id, 10);
  }

  stream.putFlag(vp_cancel_flag);

  if (!vp_cancel_flag) {
    stream.putFlag(vp_persistent_flag);

    for (auto d = 0; d < 3; ++d) {
      stream.putFloat32(vp_position[d]);
    }

    stream.putInt16(vp_rotation_qx);
    stream.putInt16(vp_rotation_qy);
    stream.putInt16(vp_rotation_qz);
    stream.putFlag(vp_center_view_flag);

    if (!vp_center_view_flag) {
      stream.putFlag(vp_left_view_flag);
    }
  }
}

auto ViewportPosition::fromViewParams(const ViewParams &viewParams) -> ViewportPosition {
  ViewportPosition vp;
  auto q = viewParams.pose.orientation;

  if (q.w() < 0) {
    q = -q;
  }

  vp.vp_cancel_flag = false;
  vp.vp_persistent_flag = true;
  vp.vp_position = viewParams.pose.position;
  vp.vp_rotation_qx = quat2unit(q.x());
  vp.vp_rotation_qy = quat2unit(q.y());
  vp.vp_rotation_qz = quat2unit(q.z());
  vp.vp_center_view_flag = true;

  return vp;
}

} // namespace TMIV::MivBitstream