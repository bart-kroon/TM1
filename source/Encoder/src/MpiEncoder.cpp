/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ISO/IEC
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

#include <TMIV/Encoder/MpiEncoder.h>

#include <TMIV/Common/Factory.h>
#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/Common/Thread.h>
#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/Formatters.h>
#include <TMIV/Packer/IPacker.h>

namespace TMIV::Encoder {
using MivBitstream::EncoderParams;

namespace {
struct MpiConfiguration {
  explicit MpiConfiguration(const Common::Json &componentNode)
      : intraPeriod{componentNode.require("intraPeriod").as<int32_t>()}
      , interPeriod{intraPeriod}
      , viewportCameraParametersSei{componentNode.require("viewportCameraParametersSei").as<bool>()}
      , viewportPositionSei{componentNode.require("viewportPositionSei").as<bool>()}
      , blockSize{componentNode.require("blockSize").as<int32_t>()}
      , textureDilation{componentNode.require("textureDilation").as<uint32_t>()}
      , transparencyDynamic{componentNode.require("transparencyDynamic").as<uint32_t>()}
      , codecGroupIdc{queryEnum(componentNode, "codecGroupIdc", "codec group",
                                MivBitstream::knownCodecGroupIdcs)}
      , toolsetIdc{queryEnum(componentNode, "toolsetIdc", "toolset",
                             MivBitstream::knownToolsetIdcs)}
      , reconstructionIdc{queryEnum(componentNode, "reconstructionIdc", "reconstruction",
                                    MivBitstream::knownReconstructionIdcs)}
      , levelIdc{queryEnum(componentNode, "levelIdc", "level", MivBitstream::knownLevelIdcs)}
      , oneV3cFrameOnly{componentNode.require("oneV3cFrameOnly").as<bool>()} {
    VERIFY(!componentNode.require("haveOccupancyVideo").as<bool>());
    VERIFY(!componentNode.require("haveGeometryVideo").as<bool>());
    VERIFY(componentNode.require("haveTextureVideo").as<bool>());
    VERIFY(componentNode.require("haveTransparencyVideo").as<bool>());

    static constexpr auto maxIntraPeriod = 32;
    VERIFY(0 < intraPeriod && intraPeriod <= maxIntraPeriod);

    if (const auto &node = componentNode.optional("interPeriod")) {
      interPeriod = node.as<int32_t>();
      VERIFY(0 < interPeriod && intraPeriod % interPeriod == 0);
    }

    VERIFY(2 <= blockSize);
    VERIFY((blockSize & (blockSize - 1)) == 0);

    for (const auto &node : componentNode.require("atlasFrameSizes").as<Common::Json::Array>()) {
      atlasFrameSizes.push_back(node.asVec<int32_t, 2>());
    }

    if (const auto &node = componentNode.optional("depthLowQualityFlag")) {
      depthLowQualityFlag = node.as<bool>();
    }

    if (const auto &subnode = componentNode.optional("ViewingSpace")) {
      viewingSpace = MivBitstream::ViewingSpace::loadFromJson(subnode, componentNode);
    }
  }

  int32_t intraPeriod;
  int32_t interPeriod;
  bool viewportCameraParametersSei;
  bool viewportPositionSei;
  int32_t blockSize;
  uint32_t textureDilation;
  uint32_t transparencyDynamic;
  std::optional<MivBitstream::ViewingSpace> viewingSpace;
  MivBitstream::PtlProfileCodecGroupIdc codecGroupIdc;
  MivBitstream::PtlProfileToolsetIdc toolsetIdc;
  MivBitstream::PtlProfileReconstructionIdc reconstructionIdc;
  MivBitstream::PtlLevelIdc levelIdc;
  bool oneV3cFrameOnly;
  std::vector<Common::Vec2i> atlasFrameSizes;
  bool depthLowQualityFlag{};
};
} // namespace

class MpiEncoder::Impl {
public:
  Impl(const Common::Json &componentNode)
      : m_config{componentNode}
      , m_packer{Common::create<Packer::IPacker>("Packer", componentNode, componentNode)} {}

  auto isStart(const MpiSourceUnit & /* unit */) -> bool {
    return ++m_lastIdx % m_config.interPeriod == 0;
  }

  void process(std::vector<MpiSourceUnit> buffer, Common::StageSource<CodableUnit> &source_) {
    if (m_firstIdx == 0) {
      prepareSequence(buffer.front().sequenceConfig);
    }

    m_mpiFrameBuffer.reserve(buffer.size());

    for (auto &unit : buffer) {
      m_mpiFrameBuffer.push_back(std::move(unit.frame));
    }

    processAccessUnit();

    auto type = m_firstIdx % m_config.intraPeriod == 0 ? MivBitstream::CodableUnitType::IDR
                                                       : MivBitstream::CodableUnitType::TRIAL;

    for (const auto &frame : m_mpiFrameBuffer) {
      source_.encode({m_params, constructAtlasFrame(frame), type});
      type = MivBitstream::CodableUnitType::SKIP;
    }

    m_mpiFrameBuffer.clear();

    m_firstIdx = m_lastIdx;
  }

  [[nodiscard]] auto maxLumaSamplesPerFrame() const { return m_maxLumaSamplesPerFrame; }

private:
  [[nodiscard]] auto log2FocLsbMinus4() const {
    // Avoid confusion but test MSB/LSB logic in decoder
    return Common::downCast<uint8_t>(std::max(4U, Common::ceilLog2(m_config.intraPeriod) + 1U) -
                                     4U);
  }

  [[nodiscard]] static auto vuiParameters(double frameRate) {
    auto numUnitsInTick = 1;
    auto timeScale = static_cast<int32_t>(numUnitsInTick * frameRate);
    LIMITATION(timeScale == numUnitsInTick * frameRate);

    auto vui = MivBitstream::VuiParameters{};
    vui.vui_num_units_in_tick(numUnitsInTick)
        .vui_time_scale(timeScale)
        .vui_poc_proportional_to_timing_flag(false)
        .vui_hrd_parameters_present_flag(false);
    vui.vui_unit_in_metres_flag(true);
    vui.coordinate_system_parameters() = {};
    return vui;
  }

  void setGiGeometry3dCoordinatesBitdepthMinus1() {
    uint8_t numBitsMinus1 = 9; // Main 10
    for (auto &vp : m_params.viewParamsList) {
      const auto size = std::max(vp.ci.ci_projection_plane_width_minus1() + 1,
                                 vp.ci.ci_projection_plane_height_minus1() + 1);
      numBitsMinus1 = std::max(numBitsMinus1, static_cast<uint8_t>(Common::ceilLog2(size) - 1));
    }
    for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
      const auto j = m_params.vps.vps_atlas_id(k);
      m_params.vps.geometry_information(j).gi_geometry_3d_coordinates_bit_depth_minus1(
          numBitsMinus1);
    }
  }

  void prepareIvau() {
    m_params.atlas.resize(m_params.vps.vps_atlas_count_minus1() + size_t{1});

    auto numBitsMinus1 =
        static_cast<uint8_t>(Common::ceilLog2(m_params.viewParamsList.front().nbMpiLayers) - 1);

    for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
      auto &atlas = m_params.atlas[k];
      const auto j = m_params.vps.vps_atlas_id(k);
      const auto &gi = m_params.vps.geometry_information(j);

      // Set ASPS parameters
      atlas.asps.asps_frame_width(m_params.vps.vps_frame_width(j))
          .asps_frame_height(m_params.vps.vps_frame_height(j))
          .asps_geometry_3d_bit_depth_minus1(gi.gi_geometry_3d_coordinates_bit_depth_minus1())
          .asps_geometry_2d_bit_depth_minus1(numBitsMinus1)
          .asps_log2_max_atlas_frame_order_cnt_lsb_minus4(log2FocLsbMinus4())
          .asps_use_eight_orientations_flag(true)
          .asps_extended_projection_enabled_flag(true)
          .asps_normal_axis_limits_quantization_enabled_flag(true)
          .asps_max_number_projections_minus1(
              static_cast<uint16_t>(m_params.viewParamsList.size() - 1))
          .asps_log2_patch_packing_block_size(Common::ceilLog2(m_config.blockSize))
          .asps_num_ref_atlas_frame_lists_in_asps(1)
          .asps_miv_extension()
          .asme_patch_constant_depth_flag(true);
    }
  }

  void prepareSequence(const MivBitstream::SequenceConfig &sequenceConfig) {
    m_params = {};
    auto &vps = m_params.vps;

    vps.profile_tier_level()
        .ptl_level_idc(m_config.levelIdc)
        .ptl_profile_codec_group_idc(m_config.codecGroupIdc)
        .ptl_profile_reconstruction_idc(m_config.reconstructionIdc)
        .ptl_profile_toolset_idc(m_config.toolsetIdc)
        .ptl_profile_toolset_constraints_information([this]() {
          return MivBitstream::ProfileToolsetConstraintsInformation{}
              .ptc_one_v3c_frame_only_flag(m_config.oneV3cFrameOnly)
              .ptc_eom_constraint_flag(true)
              .ptc_max_map_count_minus1(0)
              .ptc_max_atlas_count_minus1(0)
              .ptc_multiple_map_streams_constraint_flag(true)
              .ptc_plr_constraint_flag(true)
              .ptc_attribute_max_dimension_minus1(2)
              .ptc_attribute_max_dimension_partitions_minus1(0)
              .ptc_restricted_geometry_flag(true);
        }());

    VERIFY_MIVBITSTREAM(!m_config.atlasFrameSizes.empty());
    vps.vps_atlas_count_minus1(static_cast<uint8_t>(m_config.atlasFrameSizes.size() - 1));

    for (size_t k = 0; k < m_config.atlasFrameSizes.size(); ++k) {
      const auto j = MivBitstream::AtlasId{static_cast<uint8_t>(k)};
      vps.vps_atlas_id(k, j)
          .vps_frame_width(j, m_config.atlasFrameSizes[k].x())
          .vps_frame_height(j, m_config.atlasFrameSizes[k].y())
          .vps_attribute_video_present_flag(j, true);

      vps.attribute_information(j).ai_attribute_count(2);

      static constexpr auto textureBitDepth = 10;
      static constexpr auto transparencyBitDepth = 10;

      vps.attribute_information(j)
          .ai_attribute_type_id(0, MivBitstream::AiAttributeTypeId::ATTR_TEXTURE)
          .ai_attribute_dimension_minus1(0, 2)
          .ai_attribute_2d_bit_depth_minus1(0, textureBitDepth - 1);

      vps.attribute_information(j)
          .ai_attribute_type_id(1, MivBitstream::AiAttributeTypeId::ATTR_TRANSPARENCY)
          .ai_attribute_dimension_minus1(1, 0)
          .ai_attribute_2d_bit_depth_minus1(1, transparencyBitDepth - 1);
    }

    m_params.viewParamsList = sequenceConfig.sourceViewParams();

    m_params.viewingSpace = m_config.viewingSpace;

    if (m_config.viewportCameraParametersSei) {
      m_params.viewportCameraParameters = MivBitstream::ViewportCameraParameters::fromViewParams(
          sequenceConfig.cameraByName("viewport").viewParams);
    }

    if (m_config.viewportPositionSei) {
      m_params.viewportPosition = MivBitstream::ViewportPosition::fromViewParams(
          sequenceConfig.cameraByName("viewport").viewParams);
    }

    // Group atlases together to restrict atlas-level sub-bitstream access
    auto &gm = vps.vps_miv_extension().group_mapping();
    gm.gm_group_count(1);
    for (size_t i = 0; i < m_config.atlasFrameSizes.size(); ++i) {
      gm.gm_group_id(i, 0);
    }

    m_params.casps.casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4(log2FocLsbMinus4())
        .casps_miv_extension()
        .casme_depth_quantization_params_present_flag(true)
        .vui_parameters(vuiParameters(sequenceConfig.frameRate));

    // NOTE(FT, m55089): need to have only non basic views to allow for splitting the patches
    for (auto &v : m_params.viewParamsList) {
      v.isBasicView = false;
    }

    setGiGeometry3dCoordinatesBitdepthMinus1();

    // Set-up ASPS and AFPS
    prepareIvau();
  }

  static auto createBlockToPatchMap(size_t k, EncoderParams &params) {
    const auto &asps = params.atlas[k].asps;
    const auto &ppl = params.patchParamsList;

    const auto atlasId = params.vps.vps_atlas_id(k);
    const auto atlasBlockToPatchMapWidth = asps.asps_frame_width();
    const auto atlasBlockToPatchMapHeight = asps.asps_frame_height();

    auto btpm = Common::Frame<Common::PatchIdx>::lumaOnly(
        {atlasBlockToPatchMapWidth, atlasBlockToPatchMapHeight});

    std::fill(btpm.getPlane(0).begin(), btpm.getPlane(0).end(), Common::unusedPatchIdx);

    for (size_t p = 0; p < ppl.size(); ++p) {
      const auto &pp = ppl[p];

      if (pp.atlasId() == atlasId) {
        const auto xOrg = static_cast<size_t>(pp.atlasPatch2dPosX());
        const auto yOrg = static_cast<size_t>(pp.atlasPatch2dPosY());
        const auto atlasPatchWidthBlk = static_cast<size_t>(pp.atlasPatch2dSizeX());
        const auto atlasPatchHeightBlk = static_cast<size_t>(pp.atlasPatch2dSizeY());

        for (size_t y = 0; y < atlasPatchHeightBlk; ++y) {
          for (size_t x = 0; x < atlasPatchWidthBlk; ++x) {
            if (!asps.asps_patch_precedence_order_flag() ||
                btpm.getPlane(0)(yOrg + y, xOrg + x) == Common::unusedPatchIdx) {
              btpm.getPlane(0)(yOrg + y, xOrg + x) = static_cast<uint16_t>(p);
            }
          }
        }
      }
    }

    return btpm;
  }

  static void setTile(EncoderParams &params) {
    for (auto &atlas : params.atlas) {
      atlas.tilePartitions.clear();
      auto &tilePartition = atlas.tilePartitions.emplace_back();

      tilePartition.partitionPosX = 0;
      tilePartition.partitionPosY = 0;
      tilePartition.partitionWidth = atlas.asps.asps_frame_width();
      tilePartition.partitionHeight = atlas.asps.asps_frame_height();

      atlas.afps.atlas_frame_tile_information(TMIV::MivBitstream::AtlasFrameTileInformation{}
                                                  .afti_single_tile_in_atlas_frame_flag(true)
                                                  .afti_num_tiles_in_atlas_frame_minus1(0));

      atlas.athTemplate = MivBitstream::AtlasTileHeader{}
                              .ath_type(MivBitstream::AthType::I_TILE)
                              .ath_ref_atlas_frame_list_asps_flag(true)
                              .ath_pos_min_d_quantizer(uint8_t{});
    }
  }

  void processAccessUnit() {
    LIMITATION(m_params.viewParamsList.size() == 1);
    const auto &mpiViewParams = m_params.viewParamsList.front();
    Common::Vec2i mpiSize{
        static_cast<int32_t>(mpiViewParams.ci.ci_projection_plane_width_minus1()) + 1,
        static_cast<int32_t>(mpiViewParams.ci.ci_projection_plane_height_minus1()) + 1};

    m_params.patchParamsList.clear();

    auto aggregatedMask = Common::Frame<uint8_t>::lumaOnly(mpiSize);

    Common::FrameList<> pixelLayerIndicesPerFrame(m_mpiFrameBuffer.size(),
                                                  Common::Frame<>::lumaOnly(mpiSize));
    size_t nbActivePixels{};

    setTile(m_params);

    auto tilePartitions = std::vector<std::vector<MivBitstream::TilePartition>>{};
    auto tileSizes = std::vector<Common::SizeVector>{};

    for (const auto &atlas : m_params.atlas) {
      tilePartitions.push_back(atlas.tilePartitions);
      auto &sizes = tileSizes.emplace_back();

      for (const auto &tilePartition : atlas.tilePartitions) {
        sizes.push_back({tilePartition.partitionWidth, tilePartition.partitionHeight});
      }
    }

    m_packer->initialize(tilePartitions);
    m_packer->initialize(tileSizes, m_config.blockSize);

    for (auto layerId = 0; layerId < mpiViewParams.nbMpiLayers; ++layerId) {
      aggregatedMask.fillZero();

      for (size_t frameBufferIdx = 0; frameBufferIdx < pixelLayerIndicesPerFrame.size();
           ++frameBufferIdx) {
        const auto &frame = m_mpiFrameBuffer[frameBufferIdx];
        auto &pixelLayerIndices = pixelLayerIndicesPerFrame[frameBufferIdx];

        Common::parallelFor(frame.getPixelList().size(), [&](size_t pixelId) {
          const auto &pixel = frame.getPixelList()[pixelId];
          auto &pixelLayerIdx = pixelLayerIndices.getPlane(0)[pixelId];

          if (pixelLayerIdx < pixel.size()) {
            const auto &attribute = pixel[pixelLayerIdx];

            if (attribute.geometry == layerId) {
              aggregatedMask.getPlane(0)[pixelId] = 255;
              pixelLayerIdx++;
            }
          }
        });
      }

      nbActivePixels +=
          std::count_if(aggregatedMask.getPlane(0).begin(), aggregatedMask.getPlane(0).end(),
                        [](auto x) { return (x > 0); });

      auto patchParamsListLayer = m_packer->pack(tileSizes, {aggregatedMask},
                                                 m_params.viewParamsList, m_config.blockSize, {});

      for (auto &patchParams : patchParamsListLayer) {
        patchParams.atlasPatch3dOffsetD(layerId);

        // NOTE(BK): I do not understand why asps_2d_geometry_bit_depth_minus1 != 0, but in that
        // case we need to calculate atlasPatch3drangeD.
        const auto atlasIdx = m_params.vps.indexOf(patchParams.atlasId());
        const auto bitDepth =
            m_params.atlas[atlasIdx].asps.asps_geometry_2d_bit_depth_minus1() + 1U;
        patchParams.atlasPatch3dRangeD(Common::maxLevel(bitDepth));
      }

      std::move(patchParamsListLayer.begin(), patchParamsListLayer.end(),
                back_inserter(m_params.patchParamsList));
    }

    Common::logInfo("Aggregated luma samples per frame is {}M",
                    1e-6 * 2 * static_cast<double>(nbActivePixels));
    m_maxLumaSamplesPerFrame = std::max(m_maxLumaSamplesPerFrame, nbActivePixels);

    Common::logInfo("Packing done with nb of patches = {}", m_params.patchParamsList.size());
    m_blockToPatchMapPerAtlas.clear();

    for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
      m_blockToPatchMapPerAtlas.emplace_back(createBlockToPatchMap(k, m_params));
    }

    Common::logInfo("Block to patch map created");
  }

  static auto dilateTextureAtlas(Common::Frame<> &textureAtlas,
                                 const Common::Frame<uint8_t> &transparencyAtlas,
                                 uint32_t textureDilation) {
    const auto w = textureAtlas.getWidth();
    const auto h = textureAtlas.getHeight();

    auto texturePrev = expandTexture(textureAtlas);
    auto textureNext = expandTexture(textureAtlas);

    auto transparencyPrev = transparencyAtlas.getPlane(0);
    auto transparencyNext = transparencyPrev;

    static const std::array<Common::Vec2i, 8> offsetList = {
        Common::Vec2i({-1, -1}), Common::Vec2i({0, -1}), Common::Vec2i({1, -1}),
        Common::Vec2i({-1, 0}),  Common::Vec2i({1, 0}),  Common::Vec2i({-1, 1}),
        Common::Vec2i({0, 1}),   Common::Vec2i({1, 1})};

    for (auto iter = 0U; iter < textureDilation; iter++) {
      std::swap(transparencyPrev, transparencyNext);
      std::swap(texturePrev, textureNext);

      Common::parallelFor(w, h, [&](size_t row, size_t col) {
        int32_t cnt = 0;
        Common::Vec3f yuv{};
        if (transparencyPrev(row, col) == 0) {
          for (auto neighbour : offsetList) {
            auto x = static_cast<int32_t>(col) + neighbour.x();
            auto y = static_cast<int32_t>(row) + neighbour.y();
            if ((0 <= x) && (x < w) && (0 <= y) && (y < h)) {
              if (0 < transparencyPrev(y, x)) {
                cnt++;
                yuv += texturePrev(y, x);
              }
            }
          }
        }
        if (0 < cnt) {
          textureNext(row, col) = yuv / static_cast<float>(cnt);
          transparencyNext(row, col) = 255;
        } else {
          textureNext(row, col) = texturePrev(row, col);
          transparencyNext(row, col) = transparencyPrev(row, col);
        }
      });
    }

    return yuv420(quantizeTexture(textureNext, textureAtlas.getBitDepth()));
  }

  static auto reshapeTransparencyAtlas(Common::Frame<uint8_t> &transparencyAtlas,
                                       uint32_t transparencyDynamic) {
    const auto maxInputValue =
        static_cast<float>(Common::maxLevel(transparencyAtlas.getBitDepth()));
    const auto maxOutputValue = static_cast<float>((uint64_t{1} << transparencyDynamic) - 1);
    const auto maxStorageValue = 1023.F;

    auto transparencyAtlasReshaped = Common::Frame<>::lumaOnly(transparencyAtlas.getSize(), 10);

    std::transform(transparencyAtlas.getPlane(0).begin(), transparencyAtlas.getPlane(0).end(),
                   transparencyAtlasReshaped.getPlane(0).begin(), [&](auto v) {
                     const auto val_in = static_cast<float>(v);
                     const float val_out =
                         maxStorageValue *
                         std::floor(val_in / maxInputValue * maxOutputValue + 0.5F) /
                         maxOutputValue;

                     return static_cast<uint16_t>(val_out);
                   });

    return transparencyAtlasReshaped;
  }

  auto constructAtlasFrame(const MpiPcs::Frame &frame) -> Common::DeepFrameList {
    const auto &ppl = m_params.patchParamsList;

    Common::DeepFrameList atlasList;

    for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
      const auto atlasId = m_params.vps.vps_atlas_id(k);
      const auto frameWidth = m_params.vps.vps_frame_width(atlasId);
      const auto frameHeight = m_params.vps.vps_frame_height(atlasId);

      auto textureFrame = Common::Frame<>::yuv444({frameWidth, frameHeight}, 10);
      auto transparencyFrame = Common::Frame<uint8_t>::lumaOnly({frameWidth, frameHeight});

      textureFrame.fillNeutral();
      transparencyFrame.fillZero();

      const auto &blockToPatchMap = m_blockToPatchMapPerAtlas[k];

      Common::parallelFor(frameWidth, frameHeight, [&](size_t i, size_t j) {
        if (auto patchIdx = blockToPatchMap.getPlane(0)(i, j); patchIdx != Common::unusedPatchIdx) {
          const auto &patch = ppl[patchIdx];
          auto posInView = patch.atlasToView({static_cast<int32_t>(j), static_cast<int32_t>(i)});

          const auto &pixel = frame(posInView.y(), posInView.x());
          auto layerId = static_cast<uint16_t>(patch.atlasPatch3dOffsetD());

          auto *const iter =
              std::lower_bound(pixel.begin(), pixel.end(), layerId, [](auto pixel_, auto layerId_) {
                return pixel_.geometry < layerId_;
              });

          if (iter != pixel.end() && iter->geometry == layerId) {
            textureFrame.getPlane(0)(i, j) = iter->texture[0];
            textureFrame.getPlane(1)(i, j) = iter->texture[1];
            textureFrame.getPlane(2)(i, j) = iter->texture[2];
            transparencyFrame.getPlane(0)(i, j) = iter->transparency;
          }
        }
      });

      auto textureAtlas =
          dilateTextureAtlas(textureFrame, transparencyFrame, m_config.textureDilation);
      auto transparencyAtlas =
          reshapeTransparencyAtlas(transparencyFrame, m_config.transparencyDynamic);

      atlasList.emplace_back().texture = std::move(textureAtlas);
      atlasList.back().transparency = std::move(transparencyAtlas);
    }

    return atlasList;
  }

  MpiConfiguration m_config;
  int32_t m_firstIdx{};
  int32_t m_lastIdx{};
  std::vector<MpiPcs::Frame> m_mpiFrameBuffer;
  Common::FrameList<Common::PatchIdx> m_blockToPatchMapPerAtlas;
  std::unique_ptr<Packer::IPacker> m_packer;
  size_t m_maxLumaSamplesPerFrame{};
  EncoderParams m_params;
};

MpiEncoder::MpiEncoder(const Common::Json &componentNode) : m_impl{new Impl{componentNode}} {}

MpiEncoder::~MpiEncoder() = default;

auto MpiEncoder::maxLumaSamplesPerFrame() const -> size_t {
  return m_impl->maxLumaSamplesPerFrame();
}

auto MpiEncoder::isStart(const MpiSourceUnit &unit) -> bool { return m_impl->isStart(unit); }

void MpiEncoder::process(std::vector<MpiSourceUnit> buffer) {
  return m_impl->process(std::move(buffer), source);
}
} // namespace TMIV::Encoder
