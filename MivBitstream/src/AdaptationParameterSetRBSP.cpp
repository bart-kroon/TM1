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

#include <TMIV/MivBitstream/AdaptationParameterSetRBSP.h>

#include "verify.h"

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto operator<<(ostream &stream, const MvplUpdateMode &x) -> ostream & {
  switch (x) {
  case MvplUpdateMode::VPL_INITLIST:
    return stream << "VPL_INITLIST";
  case MvplUpdateMode::VPL_UPD_EXT:
    return stream << "VPL_UPD_EXT";
  case MvplUpdateMode::VPL_UPD_INT:
    return stream << "VPL_UPD_INT";
  case MvplUpdateMode::VPL_EXT_INT:
    return stream << "VPL_EXT_INT";
  default:
    MIVBITSTREAM_ERROR("Unknown update mode");
  }
}

auto DepthQuantization::printTo(ostream &stream, uint16_t viewId) const -> ostream & {
  VERIFY_MIVBITSTREAM(dq_quantization_law() == 0);
  stream << "dq_quantization_law[ " << viewId << " ]=" << int(dq_quantization_law())
         << "\ndq_norm_disp_low[ " << viewId << " ]=" << dq_norm_disp_low()
         << "\ndq_norm_disp_high[ " << viewId << " ]=" << dq_norm_disp_high()
         << "\ndq_depth_occ_map_threshold_default[ " << viewId
         << " ]=" << dq_depth_occ_map_threshold_default() << '\n';
  return stream;
}

auto DepthQuantization::decodeFrom(InputBitstream &bitstream) -> DepthQuantization {
  auto x = DepthQuantization{};

  const auto dq_quantization_law = bitstream.getUint8();
  VERIFY_MIVBITSTREAM(dq_quantization_law == 0);

  x.dq_norm_disp_low(bitstream.getFloat32());
  x.dq_norm_disp_high(bitstream.getFloat32());

  // TODO(BK): dq_depth_occ_map_threshold_default bit count is missing in WD4 d25
  x.dq_depth_occ_map_threshold_default(uint32_t(bitstream.readBits(10)));

  return x;
}

void DepthQuantization::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putUint8(dq_quantization_law());
  bitstream.putFloat32(dq_norm_disp_low());
  bitstream.putFloat32(dq_norm_disp_high());

  // TODO(BK): dq_depth_occ_map_threshold_default bit count is missing in WD4 d25
  bitstream.writeBits(dq_depth_occ_map_threshold_default(), 10);
}

auto MivViewParamsList::mvp_num_views_minus1() const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(!m_camera_extrinsics.empty());
  return uint16_t(m_camera_extrinsics.size() - 1);
}

auto MivViewParamsList::camera_extrinsics(const uint16_t viewId) const noexcept
    -> const CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(viewId < m_camera_extrinsics.size());
  return m_camera_extrinsics[viewId];
}

auto MivViewParamsList::camera_intrinsics(uint16_t viewId) const noexcept
    -> const CameraIntrinsics & {
  if (mvp_intrinsic_params_equal_flag()) {
    viewId = 0; // Convenience
  }
  VERIFY_MIVBITSTREAM(viewId < m_camera_intrinsics.size());
  return m_camera_intrinsics[viewId];
}

auto MivViewParamsList::depth_quantization(uint16_t viewId) const noexcept
    -> const DepthQuantization & {
  if (mvp_depth_quantization_params_equal_flag()) {
    viewId = 0; // Convenience
  }
  VERIFY_MIVBITSTREAM(viewId < m_depth_quantization.size());
  return m_depth_quantization[viewId];
}

auto MivViewParamsList::pruning_children(const uint16_t viewId) const noexcept
    -> const PruningChildren & {
  VERIFY_MIVBITSTREAM(mvp_pruning_graph_params_present_flag());
  VERIFY_MIVBITSTREAM(viewId < m_pruning_children.size());
  return m_pruning_children[viewId];
}

auto MivViewParamsList::mvp_num_views_minus1(const uint16_t value) noexcept -> MivViewParamsList & {
  m_camera_extrinsics.resize(value + 1);
  return *this;
}

auto MivViewParamsList::mvp_intrinsic_params_equal_flag(const bool value) noexcept
    -> MivViewParamsList & {
  m_mvp_intrinsic_params_equal_flag = value;
  m_camera_intrinsics.resize(value ? 1U : m_camera_extrinsics.size());
  return *this;
}

auto MivViewParamsList::mvp_depth_quantization_params_equal_flag(const bool value) noexcept
    -> MivViewParamsList & {
  m_mvp_depth_quantization_params_equal_flag = value;
  m_depth_quantization.resize(value ? 1U : m_camera_extrinsics.size());
  return *this;
}

auto MivViewParamsList::mvp_pruning_graph_params_present_flag(const bool value) noexcept
    -> MivViewParamsList & {
  m_mvp_pruning_graph_params_present_flag = value;
  m_pruning_children.resize(value ? m_camera_extrinsics.size() : 0U);
  return *this;
}

auto MivViewParamsList::camera_extrinsics(const uint16_t viewId) noexcept -> CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(viewId < m_camera_extrinsics.size());
  return m_camera_extrinsics[viewId];
}

auto MivViewParamsList::camera_intrinsics(const uint16_t viewId) noexcept -> CameraIntrinsics & {
  VERIFY_MIVBITSTREAM(viewId < m_camera_intrinsics.size());
  return m_camera_intrinsics[viewId];
}

auto MivViewParamsList::depth_quantization(const uint16_t viewId) noexcept -> DepthQuantization & {
  VERIFY_MIVBITSTREAM(viewId < m_depth_quantization.size());
  return m_depth_quantization[viewId];
}

auto MivViewParamsList::pruning_children(const uint16_t viewId) noexcept -> PruningChildren & {
  VERIFY_MIVBITSTREAM(viewId < m_pruning_children.size());
  return m_pruning_children[viewId];
}

auto operator<<(ostream &stream, const MivViewParamsList &x) -> ostream & {
  stream << "mvp_num_views_minus1=" << x.mvp_num_views_minus1() << '\n';
  for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
    x.camera_extrinsics(v).printTo(stream, v);
  }

  stream << "mvp_intrinsic_params_equal_flag=" << boolalpha << x.mvp_intrinsic_params_equal_flag()
         << '\n';
  if (x.mvp_intrinsic_params_equal_flag()) {
    x.camera_intrinsics(0).printTo(stream, 0);
  } else {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.camera_intrinsics(v).printTo(stream, v);
    }
  }

  stream << "mvp_depth_quantization_params_equal_flag=" << boolalpha
         << x.mvp_depth_quantization_params_equal_flag() << '\n';
  if (x.mvp_depth_quantization_params_equal_flag()) {
    x.depth_quantization(0).printTo(stream, 0);
  } else {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.depth_quantization(v).printTo(stream, v);
    }
  }

  stream << "mvp_pruning_graph_params_present_flag=" << boolalpha
         << x.mvp_pruning_graph_params_present_flag() << '\n';
  if (x.mvp_pruning_graph_params_present_flag()) {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.pruning_children(v).printTo(stream, v);
    }
  }
  return stream;
}

auto MivViewParamsList::operator==(const MivViewParamsList &other) const noexcept -> bool {
  return m_camera_extrinsics == other.m_camera_extrinsics &&
         m_mvp_intrinsic_params_equal_flag == other.m_mvp_intrinsic_params_equal_flag &&
         m_camera_intrinsics == other.m_camera_intrinsics &&
         m_mvp_depth_quantization_params_equal_flag ==
             other.m_mvp_depth_quantization_params_equal_flag &&
         m_depth_quantization == other.m_depth_quantization &&
         m_mvp_pruning_graph_params_present_flag == other.m_mvp_pruning_graph_params_present_flag &&
         m_pruning_children == other.m_pruning_children;
}

auto MivViewParamsList::operator!=(const MivViewParamsList &other) const noexcept -> bool {
  return !operator==(other);
}

auto MivViewParamsList::decodeFrom(InputBitstream &bitstream) -> MivViewParamsList {
  auto x = MivViewParamsList{};

  x.mvp_num_views_minus1(bitstream.getUint16());

  for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
    x.camera_extrinsics(v) = CameraExtrinsics::decodeFrom(bitstream);
  }

  x.mvp_intrinsic_params_equal_flag(bitstream.getFlag());

  if (x.mvp_intrinsic_params_equal_flag()) {
    x.camera_intrinsics() = CameraIntrinsics::decodeFrom(bitstream);
  } else {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.camera_intrinsics(v) = CameraIntrinsics::decodeFrom(bitstream);
    }
  }

  x.mvp_depth_quantization_params_equal_flag(bitstream.getFlag());

  if (x.mvp_depth_quantization_params_equal_flag()) {
    x.depth_quantization() = DepthQuantization::decodeFrom(bitstream);
  } else {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.depth_quantization(v) = DepthQuantization::decodeFrom(bitstream);
    }
  }

  x.mvp_pruning_graph_params_present_flag(bitstream.getFlag());

  if (x.mvp_pruning_graph_params_present_flag()) {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.pruning_children(v) = PruningChildren::decodeFrom(bitstream);
    }
  }
  return x;
}

void MivViewParamsList::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putUint16(mvp_num_views_minus1());

  for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
    camera_extrinsics(v).encodeTo(bitstream);
  }

  bitstream.putFlag(mvp_intrinsic_params_equal_flag());

  if (mvp_intrinsic_params_equal_flag()) {
    camera_intrinsics().encodeTo(bitstream);
  } else {
    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      camera_intrinsics(v).encodeTo(bitstream);
    }
  }

  bitstream.putFlag(mvp_depth_quantization_params_equal_flag());

  if (mvp_depth_quantization_params_equal_flag()) {
    depth_quantization().encodeTo(bitstream);
  } else {
    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      depth_quantization(v).encodeTo(bitstream);
    }
  }

  bitstream.putFlag(mvp_pruning_graph_params_present_flag());

  if (mvp_pruning_graph_params_present_flag()) {
    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      pruning_children(v).encodeTo(bitstream);
    }
  }
}

auto AdaptationParameterSetRBSP::aps_miv_view_params_list_update_mode(
    const MvplUpdateMode value) noexcept -> AdaptationParameterSetRBSP & {
  m_aps_miv_view_params_list_update_mode = value;
  return *this;
}

auto AdaptationParameterSetRBSP::aps_miv_view_params_list_update_mode() const noexcept
    -> MvplUpdateMode {
  VERIFY_MIVBITSTREAM(m_aps_miv_view_params_list_update_mode.has_value());
  return *m_aps_miv_view_params_list_update_mode;
}

auto AdaptationParameterSetRBSP::miv_view_params_list() const noexcept
    -> const MivViewParamsList & {
  VERIFY_MIVBITSTREAM(m_miv_view_params_list.has_value());
  return *m_miv_view_params_list;
}

auto AdaptationParameterSetRBSP::miv_view_params_update_extrinsics() const noexcept
    -> const MivViewParamsUpdateExtrinsics & {
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_extrinsics.has_value());
  return *m_miv_view_params_update_extrinsics;
}

auto AdaptationParameterSetRBSP::miv_view_params_update_intrinsics() const noexcept
    -> const MivViewParamsUpdateIntrinsics & {
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_intrinsics.has_value());
  return *m_miv_view_params_update_intrinsics;
}

auto operator<<(ostream &stream, const AdaptationParameterSetRBSP &x) -> ostream & {
  stream << "aps_adaptation_parameter_set_id=" << int(x.aps_adaptation_parameter_set_id())
         << "\naps_camera_params_present_flag=" << boolalpha << x.aps_camera_params_present_flag()
         << "\naps_miv_view_params_list_present_flag=" << boolalpha
         << x.aps_miv_view_params_list_present_flag() << '\n';

  if (x.aps_miv_view_params_list_present_flag()) {
    stream << "aps_miv_view_params_list_update_mode=" << x.aps_miv_view_params_list_update_mode()
           << '\n';
    switch (x.aps_miv_view_params_list_update_mode()) {
    case MvplUpdateMode::VPL_INITLIST:
      stream << x.miv_view_params_list();
      break;
    case MvplUpdateMode::VPL_UPD_EXT:
      stream << x.miv_view_params_update_extrinsics();
      break;
    case MvplUpdateMode::VPL_UPD_INT:
      stream << x.miv_view_params_update_intrinsics();
      break;
    case MvplUpdateMode::VPL_EXT_INT:
      stream << x.miv_view_params_update_extrinsics();
      stream << x.miv_view_params_update_intrinsics();
      break;
    default:
      MIVBITSTREAM_ERROR("Unknown update mode");
    }
  }
  stream << "aps_extension2_flag=" << boolalpha << x.aps_extension2_flag() << '\n';
  return stream;
}

auto AdaptationParameterSetRBSP::operator==(const AdaptationParameterSetRBSP &other) const noexcept
    -> bool {
  if (aps_adaptation_parameter_set_id() != other.aps_adaptation_parameter_set_id() ||
      aps_miv_view_params_list_present_flag() != other.aps_miv_view_params_list_present_flag()) {
    return false;
  }
  if (!aps_miv_view_params_list_present_flag()) {
    return true;
  }
  switch (aps_miv_view_params_list_update_mode()) {
  case MvplUpdateMode::VPL_INITLIST:
    return miv_view_params_list() == other.miv_view_params_list();
  case MvplUpdateMode::VPL_UPD_EXT:
    return miv_view_params_update_extrinsics() == other.miv_view_params_update_extrinsics();
  case MvplUpdateMode::VPL_UPD_INT:
    return miv_view_params_update_intrinsics() == other.miv_view_params_update_intrinsics();
  case MvplUpdateMode::VPL_EXT_INT:
    return miv_view_params_update_extrinsics() == other.miv_view_params_update_extrinsics() &&
           miv_view_params_update_intrinsics() == other.miv_view_params_update_intrinsics();
  default:
    MIVBITSTREAM_ERROR("Unknown update mode");
  }
}

auto AdaptationParameterSetRBSP::operator!=(const AdaptationParameterSetRBSP &other) const noexcept
    -> bool {
  return !operator==(other);
}

auto AdaptationParameterSetRBSP::decodeFrom(istream &stream) -> AdaptationParameterSetRBSP {
  InputBitstream bitstream{stream};

  auto x = AdaptationParameterSetRBSP{};

  x.aps_adaptation_parameter_set_id(uint8_t(bitstream.getUExpGolomb()));

  const auto aps_camera_params_present_flag = bitstream.getFlag();
  VERIFY_MIVBITSTREAM(!aps_camera_params_present_flag);

  const auto aps_extension_bit_equal_to_one = bitstream.getFlag();
  VERIFY_MIVBITSTREAM(aps_extension_bit_equal_to_one);

  x.aps_miv_view_params_list_present_flag(bitstream.getFlag());

  if (x.aps_miv_view_params_list_present_flag()) {
    x.aps_miv_view_params_list_update_mode(MvplUpdateMode(bitstream.readBits(2)));

    switch (x.aps_miv_view_params_list_update_mode()) {
    case MvplUpdateMode::VPL_INITLIST:
      x.miv_view_params_list() = MivViewParamsList::decodeFrom(bitstream);
      break;
    case MvplUpdateMode::VPL_UPD_EXT:
      x.miv_view_params_update_extrinsics() = MivViewParamsUpdateExtrinsics::decodeFrom(bitstream);
      break;
    case MvplUpdateMode::VPL_UPD_INT:
      x.miv_view_params_update_intrinsics() = MivViewParamsUpdateIntrinsics::decodeFrom(bitstream);
      break;
    case MvplUpdateMode::VPL_EXT_INT:
      x.miv_view_params_update_extrinsics() = MivViewParamsUpdateExtrinsics::decodeFrom(bitstream);
      x.miv_view_params_update_intrinsics() = MivViewParamsUpdateIntrinsics::decodeFrom(bitstream);
      break;
    }
  }

  const auto aps_extension2_flag = bitstream.getFlag();
  VERIFY_MIVBITSTREAM(!aps_extension2_flag);
  bitstream.rbspTrailingBits();

  return x;
}

void AdaptationParameterSetRBSP::encodeTo(ostream &stream) const {
  OutputBitstream bitstream{stream};

  bitstream.putUExpGolomb(aps_adaptation_parameter_set_id());

  VERIFY_MIVBITSTREAM(!aps_camera_params_present_flag());
  bitstream.putFlag(aps_camera_params_present_flag());

  constexpr auto aps_extension_bit_equal_to_one = true;
  bitstream.putFlag(aps_extension_bit_equal_to_one);

  bitstream.putFlag(aps_miv_view_params_list_present_flag());

  if (aps_miv_view_params_list_present_flag()) {
    bitstream.writeBits(uint_least64_t(aps_miv_view_params_list_update_mode()), 2);
    switch (aps_miv_view_params_list_update_mode()) {
    case MvplUpdateMode::VPL_INITLIST:
      miv_view_params_list().encodeTo(bitstream);
      break;
    case MvplUpdateMode::VPL_UPD_EXT:
      miv_view_params_update_extrinsics().encodeTo(bitstream);
      break;
    case MvplUpdateMode::VPL_UPD_INT:
      miv_view_params_update_intrinsics().encodeTo(bitstream);
      break;
    case MvplUpdateMode::VPL_EXT_INT:
      miv_view_params_update_extrinsics().encodeTo(bitstream);
      miv_view_params_update_intrinsics().encodeTo(bitstream);
      break;
    }
  }

  VERIFY_MIVBITSTREAM(!aps_extension2_flag());
  bitstream.putFlag(aps_extension2_flag());
  bitstream.rbspTrailingBits();
}
} // namespace TMIV::MivBitstream