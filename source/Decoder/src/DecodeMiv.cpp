/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#include <TMIV/Decoder/DecodeMiv.h>

#include <TMIV/Common/Bytestream.h>
#include <TMIV/Common/FlatMap.h>
#include <TMIV/Common/Frame.h>
#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/Common/verify.h>
#include <TMIV/Decoder/DecodeAtlas.h>
#include <TMIV/Decoder/DecodeAtlasSubBitstream.h>
#include <TMIV/Decoder/DecodeCommonAtlas.h>
#include <TMIV/Decoder/DecodePatchParamsList.h>
#include <TMIV/Decoder/DecodeViewParamsList.h>
#include <TMIV/Decoder/V3cUnitBuffer.h>
#include <TMIV/MivBitstream/AccessUnit.h>

#include <ctime>

#include <utility>

namespace TMIV::Decoder {
namespace {
using E = ErrorCode;

auto clockInSeconds() {
  return static_cast<double>(std::clock()) / static_cast<double>(CLOCKS_PER_SEC);
}

template <typename AAU, typename = std::enable_if_t<std::is_same_v<MivBitstream::AtlasAccessUnit,
                                                                   std::remove_const_t<AAU>>>>
auto decFrame(MivBitstream::V3cUnitHeader vuh, AAU &aau) -> auto & {
  switch (vuh.vuh_unit_type()) {
  case MivBitstream::VuhUnitType::V3C_OVD:
    return aau.decOccFrame;
  case MivBitstream::VuhUnitType::V3C_GVD:
    return aau.decGeoFrame;
  case MivBitstream::VuhUnitType::V3C_AVD:
    return aau.decAttrFrame[vuh.vuh_attribute_index()];
  case MivBitstream::VuhUnitType::V3C_PVD:
    return aau.decPckFrame;
  default:
    UNREACHABLE;
  }
}

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define MIVDECODER_CHECK(cond, err)                                                                \
  if ((cond)) {                                                                                    \
  } else {                                                                                         \
    throw Exception(err, __FILE__, __LINE__);                                                      \
  }

class MivDecoder {
public:
  MivDecoder(Common::Source<MivBitstream::V3cUnit> source, VideoDecoderFactory videoDecoderFactory,
             PtlChecker::SharedChecker checker, CommonAtlasDecoderFactory commonAtlasDecoderFactory,
             AtlasDecoderFactory atlasDecoderFactory)
      : m_inputBuffer{std::make_shared<V3cUnitBuffer>(
            std::move(source), [this](const MivBitstream::V3cUnit &vu) { onVps(vu); })}
      , m_videoDecoderFactory{std::move(videoDecoderFactory)}
      , m_checker{std::move(checker)}
      , m_commonAtlasDecoderFactory{std::move(commonAtlasDecoderFactory)}
      , m_atlasDecoderFactory{std::move(atlasDecoderFactory)} {}

  auto operator()() -> std::optional<MivBitstream::AccessUnit> {
    VERIFY(m_state == State::initial || m_state == State::decoding);

    if (m_state == State::initial) {
      if (auto vu = pullVpsV3cUnit()) {
        onVps(*vu);
        m_state = State::decoding;
        m_au.frameIdx = 0;
        m_au.foc = 0;
      } else {
        m_state = State::end;
        return std::nullopt;
      }
    } else {
      m_state = State::limbo;
      ++m_au.frameIdx;
      ++m_au.foc;
    }

    VERIFY(m_state == State::limbo || m_state == State::decoding);

    if (!std::all_of(m_vd.begin(), m_vd.end(),
                     [this](auto &kvp) { return decodeAu(kvp.key, kvp.value); })) {
      m_state = State::end;
      return std::nullopt;
    }

    VERIFY(m_state == State::decoding);

    if (m_au.foc == 0 && !m_nextVps.empty()) {
      decodeVps();
    }

    for (auto &kvp : m_cad) {
      decodeAu(kvp.key, kvp.value);
    }

    for (auto &kvp : m_ad) {
      decodeAu(kvp.key, kvp.value);
    }

    if (m_au.foc == 0) {
      stopAndStartDecoders(m_cad, commonAtlasVuhs(m_au.vps),
                           [this](MivBitstream::V3cUnitHeader vuh) {
                             return m_commonAtlasDecoderFactory(
                                 atlasSubBitstreamSource(m_inputBuffer, vuh), m_au.vps);
                           });
      stopAndStartDecoders(m_ad, atlasVuhs(m_au.vps), [this](MivBitstream::V3cUnitHeader vuh) {
        return m_atlasDecoderFactory(atlasSubBitstreamSource(m_inputBuffer, vuh), m_au.vps, vuh);
      });
      stopAndStartDecoders(m_vd, videoVuhs(m_au.vps), [this](MivBitstream::V3cUnitHeader vuh) {
        return m_videoDecoderFactory(videoSubBitstreamSource(m_inputBuffer, vuh), m_au.vps, vuh);
      });
    }

    // NOTE(BK): (Common) atlas sub-bitstreams are PTL checked by the respective decoders. For the
    // video sub-bitstreams the responsibility is with the MIV decoder, because this check needs to
    // run *after* the ASPS was decoded.
    for (const auto vuh : videoVuhs(m_au.vps)) {
      const auto &atlas = m_au.atlas[m_au.vps.indexOf(vuh.vuh_atlas_id())];
      m_checker->checkVideoFrame(vuh.vuh_unit_type(), atlas.asps, decFrame(vuh, atlas));
    }

    m_checker->checkV3cFrame(m_au);
    return m_au;
  }

private:
  enum class State {
    initial,  // before or at the start of the first call to operator ()
    limbo,    // the next MIV AU can be IRAP, non-IRAP or EOS, try decode video to find out
    decoding, // decoding an IRAP (m_au.foc == 0) or non-IRAP (0 < m_au.foc) MIV access unit
    end       // the last MIV AU has been decoded, do not call operator () again
  };

  template <typename T> struct SubDecoder {
    std::optional<T> au;
    Common::Source<T> decoder;

    void buffer() {
      if (!au && decoder) {
        au = decoder();

        if (!au) {
          decoder = nullptr;
        }
      }
    }

    template <ErrorCode missingIrap, ErrorCode expectedToBeIrap, ErrorCode misalignedFoc>
    void checkFoc(int32_t foc) {
      if (foc == 0) {
        MIVDECODER_CHECK(au, missingIrap);
        MIVDECODER_CHECK(au->foc == 0, expectedToBeIrap);
      } else {
        MIVDECODER_CHECK(!au || au->foc == 0 || foc <= au->foc, misalignedFoc);
      }
    }
  };

  template <typename T>
  using SubDecoderMap = Common::FlatMap<MivBitstream::V3cUnitHeader, SubDecoder<T>>;

  auto pullVpsV3cUnit() -> std::optional<MivBitstream::V3cUnit> {
    try {
      return (*m_inputBuffer)(MivBitstream::V3cUnitHeader::vps());
    } catch (V3cUnitBufferError &) {
      throw Exception{E::expected_vps, __FILE__, __LINE__};
    }
  }

  void onVps(const MivBitstream::V3cUnit &vu) {
    VERIFY_MIVBITSTREAM(m_state != State::end);

    m_checker->checkVuh(vu.v3c_unit_header());
    m_nextVps.push(vu.v3c_unit_payload().v3c_parameter_set());
  }

  template <typename T, typename Start>
  void stopAndStartDecoders(SubDecoderMap<T> &map,
                            const std::vector<MivBitstream::V3cUnitHeader> &next, Start &&start) {
    VERIFY(m_state == State::decoding && m_au.foc == 0);

    for (auto &kvp : map) {
      if (!Common::contains(next, kvp.key)) {
        kvp.value = {};
        Common::logInfo("[idx:{:4} foc:{:4}] Stopped decoder: {}", m_au.frameIdx, 0,
                        kvp.key.summary());
      }
    }
    for (auto vuh : next) {
      if (map.cend() !=
          std::find_if(map.cbegin(), map.cend(), [vuh](const auto &x) { return x.key == vuh; })) {
        Common::logInfo("[idx:{:4} foc:{:4}] Continued decoder: {}", m_au.frameIdx, 0,
                        vuh.summary());
      } else {
        auto &value = map[vuh];
        value.decoder = start(vuh);
        Common::logInfo("[idx:{:4} foc:{:4}] Started decoder: {}", m_au.frameIdx, 0, vuh.summary());
        VERIFY(decodeAu(vuh, value));
      }
    }
  }

  void decodeVps() {
    VERIFY(m_state == State::decoding && m_au.foc == 0 && !m_nextVps.empty());

    m_au.vps = std::move(m_nextVps.front());
    m_nextVps.pop();

    Common::logInfo(m_au.vps.summary());
    m_checker->checkAndActivateVps(m_au.vps);
    checkCapabilities();
    allocateAuBuffers();
  }

  void allocateAuBuffers() {
    m_au.atlas.resize(m_au.vps.vps_atlas_count_minus1() + size_t{1});

    for (uint8_t k = 0; k <= m_au.vps.vps_atlas_count_minus1(); ++k) {
      const auto atlasId = m_au.vps.vps_atlas_id(k);

      if (m_au.vps.vps_attribute_video_present_flag(atlasId)) {
        const auto &ai = m_au.vps.attribute_information(atlasId);
        m_au.atlas[k].decAttrFrame.resize(ai.ai_attribute_count());
      }
    }
  }

  auto decodeAu(MivBitstream::V3cUnitHeader vuh, SubDecoder<Common::DecodedFrame> &decoder)
      -> bool {
    VERIFY(m_state == State::limbo || m_state == State::decoding);

    const auto t0 = clockInSeconds();

    decoder.au = decoder.decoder();

    if (decoder.au) {
      if (decoder.au->irap) {
        MIVDECODER_CHECK(m_state == State::limbo || m_au.foc == 0, E::misaligned_video_irap);
        m_au.foc = 0;
      } else {
        MIVDECODER_CHECK(0 < m_au.foc, E::misaligned_video_irap);
      }

      m_state = State::decoding;

      const auto atlasIdx = m_au.vps.indexOf(vuh.vuh_atlas_id());
      decFrame(vuh, m_au.atlas[atlasIdx]) = std::move(*decoder.au);

      m_totalVideoDecodingTime[vuh] += clockInSeconds() - t0;
      Common::logInfo("[idx:{:4} foc:{:4}] Decoded video frame: {}", m_au.frameIdx, m_au.foc,
                      vuh.summary());
      return true;
    }
    Common::logInfo("[idx:{:4}         ] End of video stream: {}", m_au.frameIdx, vuh.summary());
    return false;
  }

  auto decodeAu(MivBitstream::V3cUnitHeader vuh, SubDecoder<CommonAtlasAccessUnit> &decoder)
      -> bool {
    VERIFY(m_state == State::decoding);

    decoder.buffer();
    decoder.checkFoc<E::missing_common_atlas_irap, E::expected_common_atlas_to_be_irap,
                     E::misaligned_common_atlas_foc>(m_au.foc);

    if (decoder.au && decoder.au->foc == m_au.foc) {
      decodeViewParamsList(*decoder.au, m_au.viewParamsList);
      decodeVui(*decoder.au, m_au.vui);
      m_au.gup = decoder.au->gup;
      m_au.vs = decoder.au->vs;
      m_au.vcp = decoder.au->vcp;
      m_au.vp = decoder.au->vp;
      m_au.casps = decoder.au->casps;
      Common::logInfo("[idx:{:4} foc:{:4}] Decoded common atlas frame: {}", m_au.frameIdx,
                      decoder.au->foc, vuh.summary());
      decoder.au = std::nullopt;
      return true;
    }
    return false;
  }

  auto decodeAu(MivBitstream::V3cUnitHeader vuh, SubDecoder<AtlasAccessUnit> &decoder) -> bool {
    VERIFY(m_state == State::decoding);

    decoder.buffer();
    decoder.checkFoc<E::missing_atlas_irap, E::expected_atlas_to_be_irap, E::misaligned_atlas_foc>(
        m_au.foc);

    if (decoder.au && decoder.au->foc == m_au.foc) {
      auto &atlas = m_au.atlas[m_au.vps.indexOf(vuh.vuh_atlas_id())];
      atlas.asps = decoder.au->asps;
      atlas.afps = decoder.au->afps;

      const auto singleTileInAtlasFrameFlag =
          atlas.afps.atlas_frame_tile_information().afti_single_tile_in_atlas_frame_flag();

      // getTile
      if (singleTileInAtlasFrameFlag) {
        atlas.tileParamsList.clear();
        MivBitstream::TilePartition tile;
        tile.partitionPosX(0);
        tile.partitionPosY(0);
        tile.partitionWidth(atlas.asps.asps_frame_width());
        tile.partitionHeight(atlas.asps.asps_frame_height());
        atlas.tileParamsList.emplace_back(tile);
      } else {
        const auto uniformPartitionFlag =
            atlas.afps.atlas_frame_tile_information().afti_uniform_partition_spacing_flag();
        const auto singlePartitionPerTileFlag =
            atlas.afps.atlas_frame_tile_information().afti_single_partition_per_tile_flag();
        int32_t numPartitionColumns = 0;
        int32_t numPartitionRows = 0;
        getPartitionInformation(atlas, uniformPartitionFlag, numPartitionColumns, numPartitionRows);
        getAtlasFameTileInformation(atlas, singlePartitionPerTileFlag, numPartitionColumns,
                                    numPartitionRows);
      }

      for (size_t tileIdx = 0; tileIdx < atlas.tileParamsList.size(); ++tileIdx) {
        auto &tile = atlas.tileParamsList[tileIdx];
        decodePatchParamsList(m_au.vps, vuh, *decoder.au, tile, tileIdx);
      }
      atlas.patchParamsList.clear();
      for (const auto &tile : atlas.tileParamsList) {
        for (auto patch : tile.partitionPatchList()) {
          atlas.patchParamsList.emplace_back(patch);
        }
      }
      requireAllPatchesWithinProjectionPlaneBounds(m_au.viewParamsList, atlas.patchParamsList);
      requireAllPatchesWithinAtlasFrameBounds(atlas.patchParamsList, atlas.asps);
      atlas.blockToPatchMap = decodeBlockToPatchMap(atlas.asps, atlas.patchParamsList);
      Common::logInfo("[idx:{:4} foc:{:4}] Decoded atlas frame: {}", m_au.frameIdx, decoder.au->foc,
                      vuh.summary());
      decoder.au = std::nullopt;
      return true;
    }
    return false;
  }

  void getPartitionInformation(const MivBitstream::AtlasAccessUnit &atlas,
                               bool uniformPartitionFlag, int32_t &numPartitionColumns,
                               int32_t &numPartitionRows) {
    std::vector<int32_t> partitionPosXList;
    std::vector<int32_t> partitionPosYList;
    std::vector<int32_t> partitionWidthList;
    std::vector<int32_t> partitionHeightList;
    partitionArray.clear();
    // width
    if (uniformPartitionFlag) {
      int32_t partitionWidth =
          (atlas.afps.atlas_frame_tile_information().afti_partition_cols_width_minus1() + 1) * 64;
      numPartitionColumns =
          static_cast<int32_t>(std::ceil(atlas.asps.asps_frame_width() / (partitionWidth * 1.0)));
      partitionPosXList.resize(numPartitionColumns);
      partitionWidthList.resize(numPartitionColumns);

      partitionPosXList[0] = 0;
      partitionWidthList[0] = partitionWidth;
      for (int32_t i = 1; i < numPartitionColumns - 1; ++i) {
        partitionPosXList[i] = partitionPosXList[i - 1] + partitionWidthList[i - 1];
        partitionWidthList[i] = partitionWidth;
      }
    } else {
      numPartitionColumns =
          (atlas.afps.atlas_frame_tile_information().afti_num_partition_columns_minus1()) + 1;
      const auto partitionColumnWidthMinus1 =
          atlas.afps.atlas_frame_tile_information().afti_partition_column_width_minus1();
      partitionPosXList.resize(numPartitionColumns);
      partitionWidthList.resize(numPartitionColumns);

      partitionPosXList[0] = 0;
      partitionWidthList[0] = (numPartitionColumns == 1) ? (atlas.asps.asps_frame_width())
                                                         : (partitionColumnWidthMinus1[0] + 1) * 64;

      for (int32_t i = 1; i < numPartitionColumns - 1; ++i) {
        partitionPosXList[i] = partitionPosXList[i - 1] + partitionWidthList[i - 1];
        partitionWidthList[i] = (partitionColumnWidthMinus1[i] + 1) * 64;
      }
    }
    if (numPartitionColumns > 1) {
      partitionPosXList[numPartitionColumns - 1] =
          partitionPosXList[numPartitionColumns - 2] + partitionWidthList[numPartitionColumns - 2];
      partitionWidthList[numPartitionColumns - 1] =
          atlas.asps.asps_frame_width() - partitionPosXList[numPartitionColumns - 1];
    }

    // height
    if (uniformPartitionFlag) {
      int32_t partitionHeight =
          (atlas.afps.atlas_frame_tile_information().afti_partition_rows_height_minus1() + 1) * 64;
      numPartitionRows =
          static_cast<int32_t>(std::ceil(atlas.asps.asps_frame_height() / (partitionHeight * 1.0)));
      partitionPosYList.resize(numPartitionRows);
      partitionHeightList.resize(numPartitionRows);

      partitionPosYList[0] = 0;
      partitionHeightList[0] = partitionHeight;
      for (int32_t j = 1; j < numPartitionRows - 1; ++j) {
        partitionPosYList[j] = partitionPosYList[j - 1] + partitionHeightList[j - 1];
        partitionHeightList[j] = partitionHeight;
      }
    } else {
      numPartitionRows =
          atlas.afps.atlas_frame_tile_information().afti_num_partition_rows_minus1() + 1;
      const auto partitionRowHeightMinus1 =
          atlas.afps.atlas_frame_tile_information().afti_partition_row_height_minus1();
      partitionPosYList.resize(numPartitionRows);
      partitionHeightList.resize(numPartitionRows);

      partitionPosYList[0] = 0;
      partitionHeightList[0] = (numPartitionRows == 1) ? (atlas.asps.asps_frame_height())
                                                       : (partitionRowHeightMinus1[0] + 1) * 64;

      for (int32_t j = 1; j < numPartitionRows - 1; ++j) {
        partitionPosYList[j] = partitionPosYList[j - 1] + partitionHeightList[j - 1];
        partitionHeightList[j] = (partitionRowHeightMinus1[j] + 1) * 64;
      }
    }
    if (numPartitionRows > 1) {
      partitionPosYList[numPartitionRows - 1] =
          partitionPosYList[numPartitionRows - 2] + partitionHeightList[numPartitionRows - 2];
      partitionHeightList[numPartitionRows - 1] =
          atlas.asps.asps_frame_height() - partitionPosYList[numPartitionRows - 1];
    }

    partitionArray.emplace_back(partitionPosXList);
    partitionArray.emplace_back(partitionPosYList);
    partitionArray.emplace_back(partitionWidthList);
    partitionArray.emplace_back(partitionHeightList);
  }

  void getAtlasFameTileInformation(MivBitstream::AtlasAccessUnit &atlas,
                                   bool singlePartitionPerTileFlag, int32_t numPartitionColumns,
                                   int32_t numPartitionRows) {
    atlas.tileParamsList.clear();
    if (!singlePartitionPerTileFlag) {
      // TODO
    } else {
      for (int32_t i = 0; i < numPartitionColumns; ++i) {
        for (int32_t j = 0; j < numPartitionRows; ++j) {
          MivBitstream::TilePartition tile;
          tile.partitionPosX(partitionArray[0][i]);
          tile.partitionPosY(partitionArray[1][j]);
          tile.partitionWidth(partitionArray[2][i]);
          tile.partitionHeight(partitionArray[3][j]);
          atlas.tileParamsList.emplace_back(tile);
        }
      }
    }
  }

  void checkCapabilities() const {
    MIVDECODER_CHECK(m_au.vps.vpsMivExtensionPresentFlag(), E::expected_miv_extension);
    MIVDECODER_CHECK(m_au.vps.vps_extension_count() ==
                         (m_au.vps.vpsPackingInformationPresentFlag() ? 2 : 1),
                     E::unsupported_vps_extension);

    auto haveVideo = false;

    for (size_t k = 0; k <= m_au.vps.vps_atlas_count_minus1(); ++k) {
      const auto j = m_au.vps.vps_atlas_id(k);
      VERIFY_MIVBITSTREAM(m_au.vps.vps_map_count_minus1(j) == 0);
      VERIFY_MIVBITSTREAM(!m_au.vps.vps_auxiliary_video_present_flag(j));

      haveVideo = haveVideo || m_au.vps.vps_occupancy_video_present_flag(j) ||
                  m_au.vps.vps_geometry_video_present_flag(j) ||
                  m_au.vps.vps_attribute_video_present_flag(j) ||
                  m_au.vps.vps_packed_video_present_flag(j);
    }
    MIVDECODER_CHECK(haveVideo, E::expected_video);
  }

  static void decodeVui(const CommonAtlasAccessUnit &au,
                        std::optional<MivBitstream::VuiParameters> &vui) {
    if (au.casps.casps_extension_present_flag() && au.casps.casps_miv_extension_present_flag()) {
      const auto &casme = au.casps.casps_miv_extension();
      if (casme.casme_vui_params_present_flag()) {
        VERIFY_MIVBITSTREAM(!vui || *vui == casme.vui_parameters());
        vui = casme.vui_parameters();
      }
    }
  }

  struct ReportTotalTime : public Common::FlatMap<MivBitstream::V3cUnitHeader, double> {
    ReportTotalTime() = default;

    ReportTotalTime(const ReportTotalTime &) = delete;
    ReportTotalTime(ReportTotalTime &&) noexcept = default;
    auto operator=(const ReportTotalTime &) -> ReportTotalTime & = delete;
    auto operator=(ReportTotalTime &&) noexcept -> ReportTotalTime & = default;

    ~ReportTotalTime() {
      for (const auto [vuh, totalTime] : *this) {
        if (0. < totalTime) {
          Common::logInfo("Total {} decoding time: {} s", vuh.summary(), totalTime);
        }
      }
    }
  };

  std::shared_ptr<V3cUnitBuffer> m_inputBuffer;
  VideoDecoderFactory m_videoDecoderFactory;
  PtlChecker::SharedChecker m_checker;
  CommonAtlasDecoderFactory m_commonAtlasDecoderFactory;
  AtlasDecoderFactory m_atlasDecoderFactory;

  State m_state{State::initial};
  std::queue<MivBitstream::V3cParameterSet> m_nextVps;

  SubDecoderMap<CommonAtlasAccessUnit> m_cad; // common atlas data
  SubDecoderMap<AtlasAccessUnit> m_ad;        // atlas data
  SubDecoderMap<Common::DecodedFrame> m_vd;   // video data
  MivBitstream::AccessUnit m_au;              // MIV access unit

  std::vector<std::vector<int32_t>> partitionArray; // partition's posX,poxY,width,height

  ReportTotalTime m_totalVideoDecodingTime;
};
} // namespace

auto decodeMiv(Common::Source<MivBitstream::V3cUnit> source,
               VideoDecoderFactory videoDecoderFactory, PtlChecker::SharedChecker checker,
               CommonAtlasDecoderFactory commonAtlasDecoderFactory,
               AtlasDecoderFactory atlasDecoderFactory)
    -> Common::Source<MivBitstream::AccessUnit> {
  return [decoder = std::make_shared<MivDecoder>(
              std::move(source), std::move(videoDecoderFactory), std::move(checker),
              std::move(commonAtlasDecoderFactory), std::move(atlasDecoderFactory))]() {
    return (*decoder)();
  };
}

auto errorStringFor(ErrorCode code) -> const char * {
  using namespace std::string_literals;

  switch (code) {
  case E::expected_atlas_to_be_irap:
    return "Expected atlas AU to be an IRAP";
  case E::expected_common_atlas_to_be_irap:
    return "Expected common atlas AU to be an IRAP";
  case E::expected_miv_extension:
    return "Expected the VPS MIV extension";
  case E::unsupported_vps_extension:
    return "The only supported VPS extensions are MIV and packed video";
  case E::expected_video:
    return "Due to potential frame skipping it is only possible to decode common atlas data or "
           "atlas data when at least one video sub-bitstream is present in the MIV bitstream";
  case E::expected_vps:
    return "Expected a VPS at the start of the V3C unit stream";
  case E::misaligned_atlas_foc:
    return "Misaligned FOC of non-IRAP AU in an atlas sub-bitstream";
  case E::misaligned_common_atlas_foc:
    return "Misaligned FOC of non-IRAP AU in the common atlas sub-bitstream";
  case E::misaligned_video_irap:
    return "Misaligned IRAP AU's in the video sub-bitstreams";
  case E::missing_atlas_irap:
    return "Atlas IRAP is missing";
  case E::missing_common_atlas_irap:
    return "Common atlas IRAP is missing";
    // There is no default case to trigger a compiler warning if a case is missing
  }
  UNREACHABLE;
}

Exception::Exception(ErrorCode code, char const *file, int32_t line)
    : Common::MivBitstreamError{errorStringFor(code)}, m_code{code}, m_file{file}, m_line{line} {}
} // namespace TMIV::Decoder
