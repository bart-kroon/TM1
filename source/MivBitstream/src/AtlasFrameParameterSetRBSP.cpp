/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#include <TMIV/MivBitstream/AtlasFrameParameterSetRBSP.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/Formatters.h>

namespace TMIV::MivBitstream {
auto operator<<(std::ostream &stream, const AtlasFrameTileInformation &x) -> std::ostream & {
  fmt::print(stream, "afti_single_tile_in_atlas_frame_flag={}\n",
             x.afti_single_tile_in_atlas_frame_flag());
  stream << "afti_signalled_tile_id_flag=" << std::boolalpha << x.afti_signalled_tile_id_flag()
         << '\n';
  if (x.afti_single_tile_in_atlas_frame_flag()) {
    return stream;
  }

  fmt::print(stream, "afti_uniform_partition_spacing_flag={}\n",
             x.afti_uniform_partition_spacing_flag());
  if (x.afti_uniform_partition_spacing_flag()) {
    fmt::print(stream, "afti_partition_cols_width_minus1={}\n",
               x.afti_partition_cols_width_minus1());
    fmt::print(stream, "afti_partition_rows_height_minus1={}\n",
               x.afti_partition_rows_height_minus1());
  } else {
    fmt::print(stream, "afti_num_partition_columns_minus1={}\n",
               x.afti_num_partition_columns_minus1());
    fmt::print(stream, "afti_num_partition_rows_minus1={}\n", x.afti_num_partition_rows_minus1());
    for (int32_t i = 0; i < x.afti_num_partition_columns_minus1(); ++i) {
      fmt::print(stream, "afti_partition_column_width_minus1={}\n",
                 x.m_afti_partition_column_width_minus1[i]);
    }
    for (int32_t i = 0; i < x.afti_num_partition_rows_minus1(); ++i) {
      fmt::print(stream, "afti_partition_row_height_minus1={}\n",
                 x.m_afti_partition_row_height_minus1[i]);
    }
  }

  fmt::print(stream, "afti_single_partition_per_tile_flag={}\n",
             x.afti_single_partition_per_tile_flag());

  if (!x.afti_single_partition_per_tile_flag()) {
    fmt::print(stream, "afti_num_tiles_in_atlas_frame_minus1={}\n",
               x.afti_num_tiles_in_atlas_frame_minus1());
    for (int32_t i = 0; i <= x.afti_num_tiles_in_atlas_frame_minus1(); ++i) {
      fmt::print(stream, "afti_top_left_partition_idx={}\n", x.m_afti_top_left_partition_idx[i]);
      fmt::print(stream, "afti_bottom_right_partition_column_offset={}\n",
                 x.m_afti_bottom_right_partition_column_offset[i]);
      fmt::print(stream, "afti_bottom_right_partition_row_offset={}\n",
                 x.m_afti_bottom_right_partition_row_offset[i]);
    }
  }
  return stream;
}

auto AtlasFrameTileInformation::decodeFrom(Common::InputBitstream &bitstream,
                                           const std::vector<AtlasSequenceParameterSetRBSP> &aspsV)
    -> AtlasFrameTileInformation {
  auto x = AtlasFrameTileInformation{};
  const auto singleTileInAtlasFrameFlag = bitstream.getFlag();
  x.afti_single_tile_in_atlas_frame_flag(singleTileInAtlasFrameFlag);
  if (!singleTileInAtlasFrameFlag) {
    const auto uniformPartitionSpacingFlag = bitstream.getFlag();
    if (uniformPartitionSpacingFlag) {
      const auto partitionColsWidthMinus1 = bitstream.getUExpGolomb<int32_t>();
      const auto partitionRowsHeightMinus1 = bitstream.getUExpGolomb<int32_t>();
      x.afti_uniform_partition_spacing_flag(uniformPartitionSpacingFlag)
          .afti_partition_cols_width_minus1(partitionColsWidthMinus1)
          .afti_partition_rows_height_minus1(partitionRowsHeightMinus1);
    } else {
      const auto numPartitionColumnsMinus1 = bitstream.getUExpGolomb<int32_t>();
      const auto numPartitionRowsMinus1 = bitstream.getUExpGolomb<int32_t>();
      auto partitionWidth = std::vector<int32_t>(numPartitionColumnsMinus1, 0);
      auto paritionHeight = std::vector<int32_t>(numPartitionRowsMinus1, 0);
      for (auto &element : partitionWidth) {
        element = bitstream.getUExpGolomb<int32_t>();
      }
      for (auto &element : paritionHeight) {
        element = bitstream.getUExpGolomb<int32_t>();
      }

      x.afti_uniform_partition_spacing_flag(uniformPartitionSpacingFlag)
          .afti_num_partition_columns_minus1(numPartitionColumnsMinus1)
          .afti_num_partition_rows_minus1(numPartitionRowsMinus1);
      x.afti_partition_column_width_minus1(std::move(partitionWidth))
          .afti_partition_row_height_minus1(std::move(paritionHeight));
    }

    const auto singlePartitionPerTileFlag = bitstream.getFlag();
    PRECONDITION(singlePartitionPerTileFlag == true);
    x.afti_single_partition_per_tile_flag(singlePartitionPerTileFlag);
    if (!singlePartitionPerTileFlag) {
      const auto numTilesInAtlasFrameMinus1 = bitstream.getUExpGolomb<int32_t>();
      const auto numTilesInAtlasFrame = static_cast<int32_t>(numTilesInAtlasFrameMinus1 + 1);
      auto topLeftPartitionIdx = std::vector<int32_t>(numTilesInAtlasFrame, 0);
      auto bottomRightPartitionColumnOffset = std::vector<int32_t>(numTilesInAtlasFrame, 0);
      auto bottomRightPartitionRowOffset = std::vector<int32_t>(numTilesInAtlasFrame, 0);
      for (int32_t i = 0; i < numTilesInAtlasFrame; i++) {
        topLeftPartitionIdx[i] = bitstream.getUExpGolomb<int32_t>();
        bottomRightPartitionColumnOffset[i] = bitstream.getUExpGolomb<int32_t>();
        bottomRightPartitionRowOffset[i] = bitstream.getUExpGolomb<int32_t>();
      }
      x.afti_num_tiles_in_atlas_frame_minus1(numTilesInAtlasFrameMinus1)
          .afti_top_left_partition_idx(std::move(topLeftPartitionIdx))
          .afti_bottom_right_partition_column_offset(std::move(bottomRightPartitionColumnOffset))
          .afti_bottom_right_partition_row_offset(std::move(bottomRightPartitionRowOffset));
    } else {
      if (uniformPartitionSpacingFlag) {
        int32_t partitionWidth = (x.afti_partition_cols_width_minus1() + 1) * 64;
        int32_t partitionHeight = (x.afti_partition_rows_height_minus1() + 1) * 64;
        auto numPartitionColumns = static_cast<int32_t>(
            std::ceil(aspsV.front().asps_frame_width() * 1.0 / (partitionWidth)));
        auto numPartitionRows = static_cast<int32_t>(
            std::ceil(aspsV.front().asps_frame_height() * 1.0 / (partitionHeight)));
        x.afti_num_tiles_in_atlas_frame_minus1(numPartitionColumns * numPartitionRows - 1);

      } else {
        int32_t numTilesInAtlasFrameMinus1 =
            (x.afti_num_partition_columns_minus1() + 1) * (x.afti_num_partition_rows_minus1() + 1) -
            1;
        x.afti_num_tiles_in_atlas_frame_minus1(numTilesInAtlasFrameMinus1);
      }
    }
  } else {
    x.afti_num_tiles_in_atlas_frame_minus1(0);
  }

  const auto afti_signalled_tile_id_flag = bitstream.getFlag();
  VERIFY_MIVBITSTREAM(!afti_signalled_tile_id_flag);

  return x;
}

void AtlasFrameTileInformation::encodeTo(
    Common::OutputBitstream &bitstream,
    const std::vector<AtlasSequenceParameterSetRBSP> &aspsV) const {
  PRECONDITION(!aspsV.empty());
  bitstream.putFlag(afti_single_tile_in_atlas_frame_flag());
  if (!afti_single_tile_in_atlas_frame_flag()) {
    bitstream.putFlag(afti_uniform_partition_spacing_flag());
    if (afti_uniform_partition_spacing_flag()) {
      bitstream.putUExpGolomb(afti_partition_cols_width_minus1());
      bitstream.putUExpGolomb(afti_partition_rows_height_minus1());
    } else {
      bitstream.putUExpGolomb(afti_num_partition_columns_minus1());
      bitstream.putUExpGolomb(afti_num_partition_rows_minus1());
      for (int32_t i = 0; i < afti_num_partition_columns_minus1(); i++) {
        // the partition of the last column can be subtracted from the sum of the previous columns
        // by the width of the atlas.
        bitstream.putUExpGolomb(m_afti_partition_column_width_minus1[i]);
      }
      for (int32_t i = 0; i < afti_num_partition_rows_minus1(); i++) {
        bitstream.putUExpGolomb(m_afti_partition_row_height_minus1[i]);
      }
    }

    bitstream.putFlag(afti_single_partition_per_tile_flag());
    if (!afti_single_partition_per_tile_flag()) {
      bitstream.putUExpGolomb(afti_num_tiles_in_atlas_frame_minus1());
      for (int32_t i = 0; i <= afti_num_tiles_in_atlas_frame_minus1(); i++) {
        bitstream.putUExpGolomb(m_afti_top_left_partition_idx[i]);
        bitstream.putUExpGolomb(m_afti_bottom_right_partition_column_offset[i]);
        bitstream.putUExpGolomb(m_afti_bottom_right_partition_row_offset[i]);
      }
    }
  }

  const auto afti_signalled_tile_id_flag = false;
  bitstream.putFlag(afti_signalled_tile_id_flag);
}

auto AfpsMivExtension::afme_inpaint_lod_scale_x_minus1(uint32_t value) noexcept
    -> AfpsMivExtension & {
  afme_inpaint_lod_enabled_flag(true);
  m_afme_inpaint_lod_scale_x_minus1 = value;
  return *this;
}

auto AfpsMivExtension::afme_inpaint_lod_scale_y_idc(uint32_t value) noexcept -> AfpsMivExtension & {
  afme_inpaint_lod_enabled_flag(true);
  m_afme_inpaint_lod_scale_y_idc = value;
  return *this;
}

auto operator<<(std::ostream &stream, const AfpsMivExtension &x) -> std::ostream & {
  if (x.m_afme_inpaint_lod_enabled_flag) {
    fmt::print(stream, "afme_inpaint_lod_enabled_flag={}\n", x.afme_inpaint_lod_enabled_flag());
    if (x.afme_inpaint_lod_enabled_flag()) {
      fmt::print(stream, "afme_inpaint_lod_scale_x_minus1={}\n",
                 x.afme_inpaint_lod_scale_x_minus1());
      fmt::print(stream, "afme_inpaint_lod_scale_y_idc={}\n", x.afme_inpaint_lod_scale_y_idc());
    }
  }
  return stream;
}

auto AfpsMivExtension::decodeFrom(Common::InputBitstream &bitstream,
                                  const AtlasFrameParameterSetRBSP &afps) -> AfpsMivExtension {
  auto x = AfpsMivExtension{};
  if (!afps.afps_lod_mode_enabled_flag()) {
    x.afme_inpaint_lod_enabled_flag(bitstream.getFlag());
    if (x.afme_inpaint_lod_enabled_flag()) {
      x.afme_inpaint_lod_scale_x_minus1(bitstream.getUExpGolomb<int32_t>());
      x.afme_inpaint_lod_scale_y_idc(bitstream.getUExpGolomb<int32_t>());
    }
  }
  return x;
}

void AfpsMivExtension::encodeTo(Common::OutputBitstream &bitstream,
                                const AtlasFrameParameterSetRBSP &afps) const {
  if (!afps.afps_lod_mode_enabled_flag()) {
    PRECONDITION(m_afme_inpaint_lod_enabled_flag);
    bitstream.putFlag(afme_inpaint_lod_enabled_flag());
    if (afme_inpaint_lod_enabled_flag()) {
      PRECONDITION(m_afme_inpaint_lod_scale_x_minus1 && m_afme_inpaint_lod_scale_y_idc);
      bitstream.putUExpGolomb(afme_inpaint_lod_scale_x_minus1());
      bitstream.putUExpGolomb(afme_inpaint_lod_scale_y_idc());
    }
  }
}

auto AtlasFrameParameterSetRBSP::afps_miv_extension() const -> AfpsMivExtension {
  VERIFY_V3CBITSTREAM(afps_miv_extension_present_flag());
  VERIFY_V3CBITSTREAM(m_afme.has_value());
  return *m_afme;
}

auto AtlasFrameParameterSetRBSP::afpsExtensionData() const -> const std::vector<bool> & {
  VERIFY_V3CBITSTREAM(afps_extension_7bits() != 0);
  VERIFY_V3CBITSTREAM(m_afpsExtensionData.has_value());
  return *m_afpsExtensionData;
}

auto AtlasFrameParameterSetRBSP::afps_miv_extension_present_flag(bool value) noexcept
    -> AtlasFrameParameterSetRBSP & {
  afps_extension_present_flag(true);
  m_afps_miv_extension_present_flag = value;
  return *this;
}

auto AtlasFrameParameterSetRBSP::afps_extension_7bits(uint8_t value) noexcept
    -> AtlasFrameParameterSetRBSP & {
  afps_extension_present_flag(true);
  m_afps_extension_7bits = value;
  return *this;
}

auto AtlasFrameParameterSetRBSP::afps_miv_extension() noexcept -> AfpsMivExtension & {
  afps_miv_extension_present_flag(true);
  if (!m_afme) {
    m_afme = AfpsMivExtension{};
  }
  return *m_afme;
}

auto AtlasFrameParameterSetRBSP::afpsExtensionData(std::vector<bool> value) noexcept
    -> AtlasFrameParameterSetRBSP & {
  PRECONDITION(afps_extension_7bits() != 0);
  m_afpsExtensionData = std::move(value);
  return *this;
}

auto operator<<(std::ostream &stream, const AtlasFrameParameterSetRBSP &x) -> std::ostream & {
  stream << "afps_atlas_frame_parameter_set_id=" << int32_t{x.afps_atlas_frame_parameter_set_id()}
         << '\n';
  stream << "afps_atlas_sequence_parameter_set_id="
         << int32_t{x.afps_atlas_sequence_parameter_set_id()} << '\n';
  stream << x.atlas_frame_tile_information();
  stream << "afps_output_flag_present_flag=" << std::boolalpha << x.afps_output_flag_present_flag()
         << '\n';
  stream << "afps_num_ref_idx_default_active_minus1="
         << int32_t{x.afps_num_ref_idx_default_active_minus1()} << '\n';
  stream << "afps_additional_lt_afoc_lsb_len=" << int32_t{x.afps_additional_lt_afoc_lsb_len()}
         << '\n';
  stream << "afps_lod_mode_enabled_flag=" << std::boolalpha << x.afps_lod_mode_enabled_flag()
         << '\n';
  stream << "afps_raw_3d_offset_bit_count_explicit_mode_flag=" << std::boolalpha
         << x.afps_raw_3d_offset_bit_count_explicit_mode_flag() << '\n';
  stream << "afps_extension_present_flag=" << std::boolalpha << x.afps_extension_present_flag()
         << '\n';
  if (x.afps_extension_present_flag()) {
    stream << "afps_miv_extension_present_flag=" << std::boolalpha
           << x.afps_miv_extension_present_flag() << '\n';
    stream << "afps_extension_7bits=" << int32_t{x.afps_extension_7bits()} << '\n';
  }
  if (x.afps_miv_extension_present_flag()) {
    stream << x.afps_miv_extension();
  }
  if (x.afps_extension_7bits() != 0) {
    for (auto bit : x.afpsExtensionData()) {
      stream << "afps_extension_data_flag=" << std::boolalpha << bit << '\n';
    }
  }
  return stream;
}

auto AtlasFrameParameterSetRBSP::operator==(const AtlasFrameParameterSetRBSP &other) const -> bool {
  if (afps_atlas_frame_parameter_set_id() != other.afps_atlas_frame_parameter_set_id() ||
      afps_atlas_sequence_parameter_set_id() != other.afps_atlas_sequence_parameter_set_id() ||
      atlas_frame_tile_information() != other.atlas_frame_tile_information() ||
      afps_output_flag_present_flag() != other.afps_output_flag_present_flag() ||
      afps_num_ref_idx_default_active_minus1() != other.afps_num_ref_idx_default_active_minus1() ||
      afps_additional_lt_afoc_lsb_len() != other.afps_additional_lt_afoc_lsb_len() ||
      afps_lod_mode_enabled_flag() != other.afps_lod_mode_enabled_flag() ||
      afps_raw_3d_offset_bit_count_explicit_mode_flag() !=
          other.afps_raw_3d_offset_bit_count_explicit_mode_flag() ||
      afps_extension_present_flag() != other.afps_extension_present_flag() ||
      afps_miv_extension_present_flag() != other.afps_miv_extension_present_flag() ||
      afps_extension_7bits() != other.afps_extension_7bits()) {
    return false;
  }
  if (afps_miv_extension_present_flag() && afps_miv_extension() != other.afps_miv_extension()) {
    return false;
  }
  if (afps_extension_7bits() != 0U && afpsExtensionData() != other.afpsExtensionData()) {
    return false;
  }
  return true;
}

auto AtlasFrameParameterSetRBSP::operator!=(const AtlasFrameParameterSetRBSP &other) const -> bool {
  return !operator==(other);
}

auto AtlasFrameParameterSetRBSP::decodeFrom(std::istream &stream,
                                            const std::vector<AtlasSequenceParameterSetRBSP> &aspsV)
    -> AtlasFrameParameterSetRBSP {
  auto x = AtlasFrameParameterSetRBSP{};
  Common::InputBitstream bitstream{stream};

  x.afps_atlas_frame_parameter_set_id(bitstream.getUExpGolomb<uint8_t>());
  VERIFY_V3CBITSTREAM(x.afps_atlas_frame_parameter_set_id() <= 63);

  x.afps_atlas_sequence_parameter_set_id(bitstream.getUExpGolomb<uint8_t>());
  const auto &asps = aspsById(aspsV, x.afps_atlas_sequence_parameter_set_id());

  x.atlas_frame_tile_information(AtlasFrameTileInformation::decodeFrom(bitstream, aspsV));

  x.afps_output_flag_present_flag(bitstream.getFlag());

  x.afps_num_ref_idx_default_active_minus1(bitstream.getUExpGolomb<uint8_t>());
  VERIFY_V3CBITSTREAM(x.afps_num_ref_idx_default_active_minus1() <= 14);

  x.afps_additional_lt_afoc_lsb_len(bitstream.getUExpGolomb<uint8_t>());
  VERIFY_V3CBITSTREAM(x.afps_additional_lt_afoc_lsb_len() <=
                      32 - (asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4));
  VERIFY_V3CBITSTREAM(asps.asps_long_term_ref_atlas_frames_flag() ||
                      x.afps_additional_lt_afoc_lsb_len() == 0);

  x.afps_lod_mode_enabled_flag(bitstream.getFlag());
  x.afps_raw_3d_offset_bit_count_explicit_mode_flag(bitstream.getFlag());
  x.afps_extension_present_flag(bitstream.getFlag());

  if (x.afps_extension_present_flag()) {
    x.afps_miv_extension_present_flag(bitstream.getFlag());
    x.afps_extension_7bits(bitstream.readBits<uint8_t>(7));
  }
  if (x.afps_miv_extension_present_flag()) {
    x.afps_miv_extension() = AfpsMivExtension::decodeFrom(bitstream, x);
  }
  if (x.afps_extension_7bits() != 0) {
    auto afpsExtensionData = std::vector<bool>{};
    while (bitstream.moreRbspData()) {
      afpsExtensionData.push_back(bitstream.getFlag());
    }
    x.afpsExtensionData(std::move(afpsExtensionData));
  }
  bitstream.rbspTrailingBits();

  return x;
}

void AtlasFrameParameterSetRBSP::encodeTo(
    std::ostream &stream, const std::vector<AtlasSequenceParameterSetRBSP> &aspsV) const {
  Common::OutputBitstream bitstream{stream};

  PRECONDITION(afps_atlas_frame_parameter_set_id() <= 63);
  bitstream.putUExpGolomb(afps_atlas_frame_parameter_set_id());

  bitstream.putUExpGolomb(afps_atlas_sequence_parameter_set_id());
  const auto &asps = aspsById(aspsV, afps_atlas_sequence_parameter_set_id());

  atlas_frame_tile_information().encodeTo(bitstream, aspsV);

  bitstream.putFlag(afps_output_flag_present_flag());

  PRECONDITION(afps_num_ref_idx_default_active_minus1() <= 14);
  bitstream.putUExpGolomb(afps_num_ref_idx_default_active_minus1());

  PRECONDITION(afps_additional_lt_afoc_lsb_len() <=
               32 - (asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4));
  PRECONDITION(asps.asps_long_term_ref_atlas_frames_flag() ||
               afps_additional_lt_afoc_lsb_len() == 0);
  bitstream.putUExpGolomb(afps_additional_lt_afoc_lsb_len());

  bitstream.putFlag(afps_lod_mode_enabled_flag());
  bitstream.putFlag(afps_raw_3d_offset_bit_count_explicit_mode_flag());
  bitstream.putFlag(afps_extension_present_flag());

  if (afps_extension_present_flag()) {
    bitstream.putFlag(afps_miv_extension_present_flag());
    bitstream.writeBits(afps_extension_7bits(), 7);
  }
  if (afps_miv_extension_present_flag()) {
    afps_miv_extension().encodeTo(bitstream, *this);
  }
  if (afps_extension_7bits() != 0) {
    for (auto bit : afpsExtensionData()) {
      bitstream.putFlag(bit);
    }
  }
  bitstream.rbspTrailingBits();
}

auto afpsById(const std::vector<AtlasFrameParameterSetRBSP> &afpsV, int32_t id)
    -> const AtlasFrameParameterSetRBSP & {
  for (const auto &x : afpsV) {
    if (id == x.afps_atlas_frame_parameter_set_id()) {
      return x;
    }
  }
  V3CBITSTREAM_ERROR("Unknown AFPS ID");
}
} // namespace TMIV::MivBitstream
