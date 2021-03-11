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

#include <TMIV/Common/Factory.h>
#include <TMIV/Common/Thread.h>
#include <TMIV/Common/verify.h>
#include <TMIV/Encoder/MpiEncoder.h>

#include <iostream>

namespace TMIV::Encoder {
namespace {
auto createBlockToPatchMap(size_t k, TMIV::MivBitstream::EncoderParams &params)
    -> Common::BlockToPatchMap {
  const auto &asps = params.atlas[k].asps;
  const auto &ppl = params.patchParamsList;

  const auto atlasId = params.vps.vps_atlas_id(k);
  const auto atlasBlockToPatchMapWidth = asps.asps_frame_width();
  const auto atlasBlockToPatchMapHeight = asps.asps_frame_height();

  auto btpm = Common::BlockToPatchMap{atlasBlockToPatchMapWidth, atlasBlockToPatchMapHeight};

  std::fill(btpm.getPlane(0).begin(), btpm.getPlane(0).end(), Common::unusedPatchId);

  for (size_t p = 0; p < ppl.size(); ++p) {
    const auto &pp = ppl[p];

    if (pp.atlasId == atlasId) {
      const auto xOrg = static_cast<size_t>(pp.atlasPatch2dPosX());
      const auto yOrg = static_cast<size_t>(pp.atlasPatch2dPosY());
      const auto atlasPatchWidthBlk = static_cast<size_t>(pp.atlasPatch2dSizeX());
      const auto atlasPatchHeightBlk = static_cast<size_t>(pp.atlasPatch2dSizeY());

      for (size_t y = 0; y < atlasPatchHeightBlk; ++y) {
        for (size_t x = 0; x < atlasPatchWidthBlk; ++x) {
          if (!asps.asps_patch_precedence_order_flag() ||
              btpm.getPlane(0)(yOrg + y, xOrg + x) == Common::unusedPatchId) {
            btpm.getPlane(0)(yOrg + y, xOrg + x) = static_cast<uint16_t>(p);
          }
        }
      }
    }
  }

  return btpm;
}

auto dilateTextureAtlas(Common::Texture444Frame &textureAtlas,
                        const Common::Transparency8Frame &transparencyAtlas,
                        unsigned textureDilation) -> Common::TextureFrame {
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

    Common::parallel_for(w, h, [&](size_t row, size_t col) {
      int cnt = 0;
      Common::Vec3f yuv{};
      if (transparencyPrev(row, col) == 0) {
        for (auto neighbour : offsetList) {
          auto x = static_cast<int>(col) + neighbour.x();
          auto y = static_cast<int>(row) + neighbour.y();
          if ((0 <= x) && (x < w) && (0 <= y) && (y < h)) {
            if (0 < transparencyPrev(y, x)) {
              cnt++;
              yuv += texturePrev(y, x);
            }
          }
        }
      }
      if (0 < cnt) {
        textureNext(row, col) = yuv / cnt;
        transparencyNext(row, col) = 255;
      } else {
        textureNext(row, col) = texturePrev(row, col);
        transparencyNext(row, col) = transparencyPrev(row, col);
      }
    });
  }

  return yuv420p(quantizeTexture(textureNext));
}

auto reshapeTransparencyAtlas(Common::Transparency8Frame &transparencyAtlas,
                              unsigned transparencyDynamic) -> Common::Transparency10Frame {
  const auto maxInputValue =
      static_cast<float>((uint64_t{1} << Common::Transparency8Frame::getBitDepth()) - 1);
  const auto maxOutputValue = static_cast<float>((uint64_t{1} << transparencyDynamic) - 1);
  const auto maxStorageValue =
      static_cast<float>((uint64_t{1} << Common::Transparency10Frame::getBitDepth()) - 1);

  auto transparencyAtlasReshaped =
      Common::Transparency10Frame{transparencyAtlas.getWidth(), transparencyAtlas.getHeight()};

  std::transform(transparencyAtlas.getPlane(0).begin(), transparencyAtlas.getPlane(0).end(),
                 transparencyAtlasReshaped.getPlane(0).begin(), [&](auto v) {
                   const auto val_in = static_cast<float>(v);
                   const float val_out =
                       maxStorageValue *
                       std::floor(val_in / maxInputValue * maxOutputValue + 0.5F) / maxOutputValue;

                   return static_cast<uint16_t>(val_out);
                 });

  return transparencyAtlasReshaped;
}
} // namespace

MpiEncoder::MpiEncoder(const Common::Json &rootNode, const Common::Json &componentNode)
    : m_packer{Common::create<Packer::IPacker>("Packer", rootNode, componentNode)} {
  // Parameters
  m_intraPeriod = rootNode.require("intraPeriod").as<int>();
  m_blockSizeDepthQualityDependent =
      rootNode.require("blockSizeDepthQualityDependent").asVec<int, 2>();

  // Enforce user-specified atlas size
  auto node = componentNode.require("overrideAtlasFrameSizes");
  for (const auto &subnode : node.as<Common::Json::Array>()) {
    m_overrideAtlasFrameSizes.push_back(subnode.asVec<int, 2>());
  }

  if (m_intraPeriod > maxIntraPeriod) {
    throw std::runtime_error("The intraPeriod parameter cannot be greater than maxIntraPeriod.");
  }

  // Testing forbidden parameters
  bool haveOccupancyVideo{rootNode.require("haveOccupancyVideo").as<bool>()};
  if (haveOccupancyVideo) {
    throw std::runtime_error("No occupancy is allowed with current version of MPI "
                             "encoder. Please use haveOccupancyVideo = false !!!");
  }
  bool haveGeometryVideo{rootNode.require("haveGeometryVideo").as<bool>()};
  if (haveGeometryVideo) {
    throw std::runtime_error("No geometry is allowed with current version of MPI "
                             "encoder. Please use haveGeometryVideo = false !!!");
  }

  // MPI-specific parameters
  m_textureDilation = componentNode.require("TextureDilation").as<int>();
  m_transparencyDynamic = componentNode.require("TransparencyDynamic").as<int>();
}

void MpiEncoder::prepareSequence(MivBitstream::EncoderParams sourceParams) {
  m_blockSize = m_blockSizeDepthQualityDependent[static_cast<size_t>(
      sourceParams.casme().casme_depth_low_quality_flag())];
  VERIFY(2 <= m_blockSize);
  VERIFY((m_blockSize & (m_blockSize - 1)) == 0);

  // Create IVS with VPS with right number of atlases but copy other parts from input IVS
  m_params = MivBitstream::EncoderParams{m_overrideAtlasFrameSizes, 10, 0, 0, 10};

  m_params.vme() = sourceParams.vme();

  // Group atlases together to restrict atlas-level sub-bitstream access
  auto &gm = m_params.vme().group_mapping();
  gm.gm_group_count(1);
  for (size_t i = 0; i < m_overrideAtlasFrameSizes.size(); ++i) {
    gm.gm_group_id(i, 0);
  }

  m_params.viewParamsList = sourceParams.viewParamsList;
  m_params.frameRate = sourceParams.frameRate;
  m_params.lengthsInMeters = sourceParams.lengthsInMeters;
  m_params.maxEntityId = sourceParams.maxEntityId;
  m_params.casps.casps_extension_present_flag(true)
      .casps_miv_extension_present_flag(true)
      .casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4(log2FocLsbMinus4())
      .casps_miv_extension()
      .casme_depth_quantization_params_present_flag(true)
      .casme_vui_params_present_flag(true)
      .vui_parameters(vuiParameters());
  m_params.viewingSpace = sourceParams.viewingSpace;

  // NOTE(FT)/m55089 implementation : need to have only non basic views to allow for splitting the
  // patches
  for (auto &v : m_params.viewParamsList) {
    v.isBasicView = false;
  }

  setGiGeometry3dCoordinatesBitdepthMinus1();

  // Set-up ASPS and AFPS
  prepareIvau();
}

auto MpiEncoder::processAccessUnit(int firstFrameId, int lastFrameId)
    -> const MivBitstream::EncoderParams & {
  const auto &mpiViewParams = m_params.viewParamsList.front();
  Common::Vec2i mpiSize{static_cast<int>(mpiViewParams.ci.ci_projection_plane_width_minus1()) + 1,
                        static_cast<int>(mpiViewParams.ci.ci_projection_plane_height_minus1()) + 1};

  // Mask aggregation
  Common::MaskList aggregatedMaskList(mpiViewParams.nbMpiLayers,
                                      Common::Mask{mpiSize.x(), mpiSize.y()});

  for (auto &aggregatedMask : aggregatedMaskList) {
    aggregatedMask.fillZero();
  }

  m_mpiFrameBuffer.clear();

  for (int frameIndex = firstFrameId; frameIndex < lastFrameId; frameIndex++) {
    auto frame = readFrame(frameIndex);

    Common::parallel_for(frame.getPixelList().size(), [&](size_t pixelId) {
      const auto &pixel = frame.getPixelList()[pixelId];

      for (const auto &attribute : pixel) {
        aggregatedMaskList[attribute.geometry].getPlane(0)[pixelId] = 255;
      }
    });

    m_mpiFrameBuffer.emplace_back(std::move(frame));
  }

  std::cout << "Mask aggregation done" << std::endl;

  size_t nbActivePixels{};

  for (const auto &aggregatedMask : aggregatedMaskList) {
    nbActivePixels +=
        std::count_if(aggregatedMask.getPlane(0).begin(), aggregatedMask.getPlane(0).end(),
                      [](auto x) { return (x > 0); });
  }

  std::cout << "Aggregated luma samples per frame is " << (1e-6 * 2 * nbActivePixels) << "M\n";
  m_maxLumaSamplesPerFrame = std::max(m_maxLumaSamplesPerFrame, nbActivePixels);

  // Patch list
  MivBitstream::ViewParamsList viewList(
      std::vector<MivBitstream::ViewParams>(mpiViewParams.nbMpiLayers, mpiViewParams));

  auto patchParamsList =
      m_packer->pack(m_params.atlasSizes(), aggregatedMaskList, viewList, m_blockSize);

  for (auto &patchParams : patchParamsList) {
    patchParams.atlasPatch3dOffsetD(patchParams.atlasPatchProjectionId());
    patchParams.atlasPatchProjectionId(0);
  }

  m_params.patchParamsList = std::move(patchParamsList);

  std::cout << "Packing done with nb of patches = "
            << static_cast<int>(m_params.patchParamsList.size()) << std::endl;

  m_blockToPatchMapPerAtlas.clear();

  for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    m_blockToPatchMapPerAtlas.emplace_back(createBlockToPatchMap(k, m_params));
  }

  std::cout << "Block to patch map created" << std::endl;

  return m_params;
}

auto MpiEncoder::popAtlas() -> Common::MVD10Frame {
  const auto &ppl = m_params.patchParamsList;
  const auto &mpiFrame = m_mpiFrameBuffer.front();
  Common::MVD10Frame atlasList;

  for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_params.vps.vps_atlas_id(k);
    const auto frameWidth = m_params.vps.vps_frame_width(j);
    const auto frameHeight = m_params.vps.vps_frame_height(j);

    auto textureFrame = Common::Texture444Frame{frameWidth, frameHeight};
    auto transparencyFrame = Common::Transparency8Frame{frameWidth, frameHeight};

    textureFrame.fillNeutral();
    transparencyFrame.fillZero();

    const auto &blockToPatchMap = m_blockToPatchMapPerAtlas[k];

    Common::parallel_for(frameWidth, frameHeight, [&](size_t i, size_t j) {
      if (auto patchId = blockToPatchMap.getPlane(0)(i, j); patchId != Common::unusedPatchId) {
        const auto &patch = ppl[patchId];
        auto posInView = patch.atlasToView({static_cast<int>(j), static_cast<int>(i)});

        const auto &pixel = mpiFrame(posInView.y(), posInView.x());
        auto layerId = static_cast<uint16_t>(patch.atlasPatch3dOffsetD());

        const auto iter =
            std::lower_bound(pixel.begin(), pixel.end(), layerId,
                             [](auto pixel_, auto layerId_) { return pixel_.geometry < layerId_; });

        if (iter != pixel.end() && iter->geometry == layerId) {
          textureFrame.getPlane(0)(i, j) = iter->texture[0];
          textureFrame.getPlane(1)(i, j) = iter->texture[1];
          textureFrame.getPlane(2)(i, j) = iter->texture[2];
          transparencyFrame.getPlane(0)(i, j) = iter->transparency;
        }
      }
    });

    auto textureAtlas = dilateTextureAtlas(textureFrame, transparencyFrame, m_textureDilation);
    auto transparencyAtlas = reshapeTransparencyAtlas(transparencyFrame, m_transparencyDynamic);

    atlasList.emplace_back(std::move(textureAtlas), Common::Depth10Frame{},
                           Common::Occupancy10Frame{}, std::move(transparencyAtlas));
  }

  m_mpiFrameBuffer.pop_front();

  incrementFoc();

  return atlasList;
}

auto MpiEncoder::vuiParameters() const -> MivBitstream::VuiParameters {
  auto numUnitsInTick = 1;
  auto timeScale = static_cast<int>(numUnitsInTick * m_params.frameRate);
  LIMITATION(timeScale == numUnitsInTick * m_params.frameRate);

  auto vui = MivBitstream::VuiParameters{};
  vui.vui_timing_info_present_flag(true)
      .vui_num_units_in_tick(numUnitsInTick)
      .vui_time_scale(timeScale)
      .vui_poc_proportional_to_timing_flag(false)
      .vui_hrd_parameters_present_flag(false);
  vui.vui_unit_in_metres_flag(true);
  vui.vui_coordinate_system_parameters_present_flag(true).coordinate_system_parameters() = {};
  return vui;
}

void MpiEncoder::setGiGeometry3dCoordinatesBitdepthMinus1() {
  uint8_t numBitsMinus1 = 9; // Main 10
  for (auto &vp : m_params.viewParamsList) {
    const auto size = std::max(vp.ci.ci_projection_plane_width_minus1() + 1,
                               vp.ci.ci_projection_plane_height_minus1() + 1);
    numBitsMinus1 = std::max(numBitsMinus1, static_cast<uint8_t>(Common::ceilLog2(size) - 1));
  }
  for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_params.vps.vps_atlas_id(k);
    m_params.vps.geometry_information(j).gi_geometry_3d_coordinates_bit_depth_minus1(numBitsMinus1);
  }
}

void MpiEncoder::prepareIvau() {
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
        .asps_log2_patch_packing_block_size(Common::ceilLog2(m_blockSize));

    atlas.asme().asme_max_entity_id(m_params.maxEntityId);

    // Signalling patch_constant_flag requires ASME to be present
    if (m_params.vps.vps_miv_extension_present_flag()) {
      atlas.asme().asme_patch_constant_depth_flag(true);
    }

    // Set ATH parameters
    atlas.ath.ath_ref_atlas_frame_list_asps_flag(true);
    atlas.ath.ath_pos_min_d_quantizer(uint8_t(0));
  }
}

auto MpiEncoder::log2FocLsbMinus4() const -> uint8_t {
  // Avoid confusion but test MSB/LSB logic in decoder
  return Common::downCast<uint8_t>(std::max(4U, Common::ceilLog2(m_intraPeriod) + 1U) - 4U);
}

void MpiEncoder::incrementFoc() {
  const auto &atlas0 = m_params.atlas.front();
  const auto log2FocLsb = atlas0.asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4U;
  auto focLsb = atlas0.ath.ath_atlas_frm_order_cnt_lsb();
  if (++focLsb >> log2FocLsb == 1U) {
    focLsb = 0;
  }
  for (auto &atlas : m_params.atlas) {
    atlas.ath.ath_atlas_frm_order_cnt_lsb(focLsb);
  }
}

} // namespace TMIV::Encoder
