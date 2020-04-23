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

#include <TMIV/AtlasConstructor/AtlasConstructor.h>

#include "Cluster.h"
#include <TMIV/Common/Factory.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>

#include <cassert>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::AtlasConstructor {
namespace {
void runtimeCheck(bool cond, const char *what) {
  if (!cond) {
    throw runtime_error(what);
  }
}
} // namespace

constexpr auto neutralChroma = TextureFrame::neutralColor();

AtlasConstructor::AtlasConstructor(const Json &rootNode, const Json &componentNode) {
  // Components
  m_pruner = Factory<IPruner>::getInstance().create("Pruner", rootNode, componentNode);
  m_aggregator = Factory<IAggregator>::getInstance().create("Aggregator", rootNode, componentNode);
  m_packer = Factory<IPacker>::getInstance().create("Packer", rootNode, componentNode);

  // Parameters
  const auto numGroups = rootNode.require("numGroups").asInt();
  m_blockSize = rootNode.require("blockSize").asInt();
  const auto maxLumaSampleRate = rootNode.require("maxLumaSampleRate").asDouble();
  const auto maxLumaPictureSize = rootNode.require("maxLumaPictureSize").asInt();
  const auto maxAtlases = rootNode.require("maxAtlases").asInt();
  m_geometryScaleEnabledFlag = rootNode.require("geometryScaleEnabledFlag").asBool();

  // Check parameters
  runtimeCheck(1 <= numGroups, "numGroups should be at least one");
  runtimeCheck(2 <= m_blockSize, "blockSize should be at least two");
  runtimeCheck((m_blockSize & (m_blockSize - 1)) == 0, "blockSize should be a power of two");
  if (maxLumaSampleRate == 0) {
    runtimeCheck(maxLumaPictureSize == 0 && maxAtlases == 0,
                 "Either specify all constraints or none");
  } else {
    runtimeCheck(maxLumaPictureSize > 0 && maxAtlases > 0,
                 "Either specify all constraints or none");
    runtimeCheck(numGroups <= maxAtlases, "There should be at least one atlas per group");
  }

  // Translate parameters to concrete constraints
  const auto lumaSamplesPerAtlasSample = m_geometryScaleEnabledFlag ? 1.25 : 2.;
  m_maxBlockRate = maxLumaSampleRate / (numGroups * lumaSamplesPerAtlasSample * sqr(m_blockSize));
  m_maxBlocksPerAtlas = maxLumaPictureSize / sqr(m_blockSize);
  m_maxAtlases = maxAtlases / numGroups;

  // Read the entity encoding range if exisited
  if (auto subnode = componentNode.optional("EntityEncodeRange")) {
    m_EntityEncRange = subnode.asIntVector<2>();
  }

  if (rootNode.require("intraPeriod").asInt() > maxIntraPeriod) {
    throw runtime_error("The intraPeriod parameter cannot be greater than maxIntraPeriod.");
  }
}

// Calculate atlas frame sizes [MPEG/M52994 v2]
auto AtlasConstructor::calculateNominalAtlasFrameSizes(
    const IvSequenceParams &ivSequenceParams) const -> SizeVector {
  if (m_maxBlockRate == 0) {
    // No constraints: one atlas per transport view
    auto result = SizeVector(ivSequenceParams.viewParamsList.size());
    transform(cbegin(ivSequenceParams.viewParamsList), cend(ivSequenceParams.viewParamsList),
              begin(result), [](const ViewParams &x) { return x.ci.projectionPlaneSize(); });
    return result;
  }

  // Translate block rate into a maximum number of blocks
  const auto maxBlocks = int(m_maxBlockRate / ivSequenceParams.frameRate);

  // Calculate the number of atlases
  auto numAtlases = (maxBlocks + m_maxBlocksPerAtlas - 1) / m_maxBlocksPerAtlas;
  if (numAtlases > m_maxAtlases) {
    cout << "The maxAtlases constraint is a limiting factor.\n";
    numAtlases = m_maxAtlases;
  }

  // Calculate the number of blocks per atlas
  auto maxBlocksPerAtlas = maxBlocks / numAtlases;
  if (maxBlocksPerAtlas > m_maxBlocksPerAtlas) {
    cout << "The maxLumaPictureSize constraint is a limiting factor.\n";
    maxBlocksPerAtlas = m_maxBlocksPerAtlas;
  }

  // Take the smallest reasonable width
  const auto viewGridSize = calculateViewGridSize(ivSequenceParams);
  const auto atlasGridWidth = viewGridSize.x();
  const auto atlasGridHeight = maxBlocksPerAtlas / atlasGridWidth;

  // Warn if the aspect ratio is outside of HEVC limits (unlikely)
  if (atlasGridWidth * 8 < atlasGridHeight || atlasGridHeight * 8 < atlasGridWidth) {
    cout << "WARNING: Atlas aspect ratio is outside of HEVC general tier and level limits\n";
  }

  return SizeVector(numAtlases, {atlasGridWidth * m_blockSize, atlasGridHeight * m_blockSize});
}

auto AtlasConstructor::calculateViewGridSize(const IvSequenceParams &ivSequenceParams) const
    -> Vec2i {
  int x{};
  int y{};

  for (const auto &viewParams : ivSequenceParams.viewParamsList) {
    x = max(x, (viewParams.ci.ci_projection_plane_width_minus1() + m_blockSize) / m_blockSize);
    y = max(y, (viewParams.ci.ci_projection_plane_height_minus1() + m_blockSize) / m_blockSize);
  }

  return {x, y};
}

auto AtlasConstructor::prepareSequence(IvSequenceParams ivSequenceParams, vector<bool> isBasicView)
    -> const IvSequenceParams & {
  m_inIvSequenceParams = move(ivSequenceParams);

  // Do we also have texture or only geometry?
  assert(m_inIvSequenceParams.vps.vps_atlas_count_minus1() == 0);
  const auto &ai = m_inIvSequenceParams.vps.attribute_information(0);
  auto const haveTexture =
      ai.ai_attribute_count() >= 1 && ai.ai_attribute_type_id(0) == AiAttributeTypeId::ATTR_TEXTURE;

  // Calculate nominal atlas frame sizes
  const auto atlasFrameSizes = calculateNominalAtlasFrameSizes(m_inIvSequenceParams);
  cout << "Nominal atlas frame sizes: { ";
  for (auto &size : atlasFrameSizes) {
    cout << ' ' << size;
  }
  cout << " }\n";

  // Create IVS with VPS with right number of atlases but copy other parts from input IVS
  m_outIvSequenceParams = IvSequenceParams{atlasFrameSizes, haveTexture};
  m_outIvSequenceParams.msp() = m_inIvSequenceParams.msp();
  m_outIvSequenceParams.viewParamsList = m_inIvSequenceParams.viewParamsList;
  m_outIvSequenceParams.viewingSpace = m_inIvSequenceParams.viewingSpace;
  m_outIvSequenceParams.frameRate = m_inIvSequenceParams.frameRate;
  setGiGeometry3dCoordinatesBitdepthMinus1();

  m_isBasicView = move(isBasicView);

  // Register pruning relation
  m_pruner->registerPruningRelation(m_outIvSequenceParams, m_isBasicView);

  // Read max_entities
  m_maxEntities = m_outIvSequenceParams.msp().msp_max_entities_minus1() + 1;

  // Turn on occupancy coding for all views (if msp_max_entities_minus1>0) otherwise for partial
  // views only
  for (size_t viewId = 0; viewId < m_outIvSequenceParams.viewParamsList.size(); ++viewId) {
    if (!m_isBasicView[viewId] || m_maxEntities > 1) {
      m_outIvSequenceParams.viewParamsList[viewId].hasOccupancy = true;
    }
  }

  return m_outIvSequenceParams;
}

void AtlasConstructor::setGiGeometry3dCoordinatesBitdepthMinus1() {
  uint8_t numBitsMinus1 = 9; // Main 10
  for (auto &vp : m_outIvSequenceParams.viewParamsList) {
    const auto size = max(vp.ci.ci_projection_plane_width_minus1() + 1,
                          vp.ci.ci_projection_plane_height_minus1() + 1);
    numBitsMinus1 = max(numBitsMinus1, static_cast<uint8_t>(ceilLog2(size) - 1));
  }
  for (uint8_t atlasId = 0; atlasId <= m_outIvSequenceParams.vps.vps_atlas_count_minus1();
       ++atlasId) {
    m_outIvSequenceParams.vps.geometry_information(atlasId)
        .gi_geometry_3d_coordinates_bitdepth_minus1(numBitsMinus1);
  }
}

void AtlasConstructor::prepareAccessUnit(MivBitstream::IvAccessUnitParams ivAccessUnitParams) {
  m_ivAccessUnitParams = ivAccessUnitParams;

  const auto numOfCam = m_inIvSequenceParams.viewParamsList.size();

  for (size_t c = 0; c < numOfCam; c++) {
    Mat<bitset<maxIntraPeriod>> nonAggMask;
    int H = m_inIvSequenceParams.viewParamsList[c].ci.projectionPlaneSize().y();
    int W = m_inIvSequenceParams.viewParamsList[c].ci.projectionPlaneSize().x();

    nonAggMask.resize(H, W);
    for (int h = 0; h < H; h++) {
      for (int w = 0; w < W; w++) {
        nonAggMask(h, w) = 0;
      }
    }
    m_nonAggregatedMask.push_back(nonAggMask);
  }

  m_viewBuffer.clear();
  m_aggregatedEntityMask.clear();
  m_aggregator->prepareAccessUnit();
}

auto AtlasConstructor::yuvSampler(const EntityMapList &in) -> vector<Frame<YUV420P16>> {
  vector<Frame<YUV420P16>> outYuvAll;
  for (const auto &viewId : in) {
    Frame<YUV420P16> outYuv(int(viewId.getWidth()), int(viewId.getHeight()));
    const auto width = viewId.getWidth();
    const auto height = viewId.getHeight();
    int step = 1;
    for (int k = 0; k < 3; ++k) {
      if (k != 0) {
        step = 2;
      }
      int rowIndex = 0;
      for (int i = 0; i != height; i = i + step) {
        int colIndex = 0;
        for (int j = 0; j != width; j = j + step) {
          outYuv.getPlane(k)(rowIndex, colIndex) = viewId.getPlane(0)(i, j);
          colIndex++;
        }
        rowIndex++;
      }
    }
    outYuvAll.push_back(outYuv);
  }
  return outYuvAll;
}

void AtlasConstructor::mergeMasks(MaskList &mergedMasks, MaskList masks) {
  for (size_t viewId = 0; viewId < mergedMasks.size(); viewId++) {
    for (size_t i = 0; i < mergedMasks[viewId].getPlane(0).size(); i++) {
      if (masks[viewId].getPlane(0)[i] != uint8_t(0)) {
        mergedMasks[viewId].getPlane(0)[i] = masks[viewId].getPlane(0)[i];
      }
    }
  }
}

void AtlasConstructor::updateMasks(const MVD16Frame &views, MaskList &masks) {
  for (size_t viewId = 0; viewId < views.size(); viewId++) {
    for (size_t i = 0; i < masks[viewId].getPlane(0).size(); i++) {
      if ((views[viewId].texture.getPlane(0)[i] == neutralChroma) &&
          (views[viewId].depth.getPlane(0)[i] == uint16_t(0))) {
        masks[viewId].getPlane(0)[i] = uint8_t(0);
      }
    }
  }
}

void AtlasConstructor::aggregateEntityMasks(MaskList &Masks, uint16_t entityId) {
  if (int(m_aggregatedEntityMask.size()) < m_EntityEncRange[1] - m_EntityEncRange[0]) {
    m_aggregatedEntityMask.push_back(Masks);
  } else {
    for (size_t i = 0; i < Masks.size(); i++) {
      transform(m_aggregatedEntityMask[entityId - m_EntityEncRange[0]][i].getPlane(0).begin(),
                m_aggregatedEntityMask[entityId - m_EntityEncRange[0]][i].getPlane(0).end(),
                Masks[i].getPlane(0).begin(),
                m_aggregatedEntityMask[entityId - m_EntityEncRange[0]][i].getPlane(0).begin(),
                [](auto v1, auto v2) { return max(v1, v2); });
    }
  }
}

auto AtlasConstructor::entitySeparator(const MVD16Frame &transportViews, uint16_t entityId)
    -> MVD16Frame {
  // Initalize entityViews
  MVD16Frame entityViews;
  for (const auto &transportView : transportViews) {
    TextureDepth16Frame entityView = {
        TextureFrame(transportView.texture.getWidth(), transportView.texture.getHeight()),
        Depth16Frame(transportView.depth.getWidth(), transportView.depth.getHeight())};
    entityViews.push_back(move(entityView));
  }
  EntityMapList entityMaps;
  for (const auto &transportView : transportViews) {
    entityMaps.push_back(transportView.entities);
  }

  auto entityMapsYUV = yuvSampler(entityMaps);

  for (size_t viewId = 0; viewId < transportViews.size(); viewId++) {
    for (int planeId = 0; planeId < transportViews[viewId].texture.getNumberOfPlanes();
         planeId++) {                                                          //
      std::transform(transportViews[viewId].texture.getPlane(planeId).begin(), // i's
                     transportViews[viewId].texture.getPlane(planeId).end(),   //
                     entityMapsYUV[viewId].getPlane(planeId).begin(),          // j's
                     entityViews[viewId].texture.getPlane(planeId).begin(),    // result
                     [=](auto i, auto j) { return (j == entityId) ? i : neutralChroma; });
    }
    std::transform(transportViews[viewId].depth.getPlane(0).begin(), // i's
                   transportViews[viewId].depth.getPlane(0).end(),   //
                   entityMaps[viewId].getPlane(0).begin(),           // j's
                   entityViews[viewId].depth.getPlane(0).begin(),    // result
                   [=](auto i, auto j) { return (j == entityId) ? i : uint16_t(0); });
  }

  return entityViews;
}

void AtlasConstructor::pushFrame(MVD16Frame transportViews) {
  MaskList mergedMasks;
  if (m_maxEntities > 1) {
    // Initalization
    MVD16Frame transportEntityViews;
    MaskList masks;
    for (auto &transportView : transportViews) {
      Mask entityMergedMask(transportView.texture.getWidth(), transportView.texture.getHeight());

      fill(entityMergedMask.getPlane(0).begin(), entityMergedMask.getPlane(0).end(), uint8_t(0));

      mergedMasks.push_back(move(entityMergedMask));
    }

    SizeVector m_viewSizes = m_inIvSequenceParams.viewParamsList.viewSizes();

    for (auto entityId = m_EntityEncRange[0]; entityId < m_EntityEncRange[1]; entityId++) {
      cout << "Processing entity " << entityId << '\n';

      // Entity Separator
      transportEntityViews = entitySeparator(transportViews, entityId);

      // Pruning
      masks = m_pruner->prune(m_inIvSequenceParams, transportEntityViews, m_isBasicView);

      // updating the pruned basic masks for entities and filter other masks.
      updateMasks(transportEntityViews, masks);

      // Aggregate Entity Masks
      aggregateEntityMasks(masks, entityId);

      // Entity Mask Merging
      mergeMasks(mergedMasks, masks);
    }
  } else {
    mergedMasks = m_pruner->prune(m_inIvSequenceParams, transportViews, m_isBasicView);
  }

  const auto frame = m_viewBuffer.size();
  for (size_t view = 0; view < mergedMasks.size(); ++view) {
    int H = transportViews[view].texture.getHeight();
    int W = transportViews[view].texture.getWidth();
    for (int h = 0; h < H; h++) {
      for (int w = 0; w < W; w++) {
        if (mergedMasks[view].getPlane(0)(h, w) != 0) {
          m_nonAggregatedMask[view](h, w)[frame] = true;
        }
      } // w
    }   // h
  }     // view

  // Aggregation
  m_viewBuffer.push_back(move(transportViews));
  m_aggregator->pushMask(mergedMasks);
}

auto AtlasConstructor::completeAccessUnit() -> const IvAccessUnitParams & {
  // Aggregated mask
  m_aggregator->completeAccessUnit();
  const MaskList &aggregatedMask = m_aggregator->getAggregatedMask();

  // Print statistics the same way the HierarchicalPruner does
  const auto lumaSamplesPerFrame = accumulate(
      aggregatedMask.begin(), aggregatedMask.end(), size_t{}, [](size_t sum, const auto &mask) {
        return sum + 2 * count_if(mask.getPlane(0).begin(), mask.getPlane(0).end(),
                                  [](auto x) { return x > 0; });
      });
  cout << "Aggregated luma samples per frame is " << (1e-6 * lumaSamplesPerFrame) << "M\n";
  m_maxLumaSamplesPerFrame = max(m_maxLumaSamplesPerFrame, lumaSamplesPerFrame);

  // Set atlas parameters
  m_ivAccessUnitParams.atlas.resize(m_outIvSequenceParams.vps.vps_atlas_count_minus1() + 1);

  for (uint8_t i = 0; i <= m_outIvSequenceParams.vps.vps_atlas_count_minus1(); ++i) {
    auto &atlas = m_ivAccessUnitParams.atlas[i];
    const auto frameWidth = m_outIvSequenceParams.vps.vps_frame_width(i);
    const auto frameHeight = m_outIvSequenceParams.vps.vps_frame_height(i);

    // Set ASPS parameters
    atlas.asps.asps_frame_width(frameWidth)
        .asps_frame_height(frameHeight)
        .asps_use_eight_orientations_flag(true)
        .asps_extended_projection_enabled_flag(true)
        .asps_normal_axis_limits_quantization_enabled_flag(true)
        .asps_max_projections_minus1(uint16_t(m_outIvSequenceParams.viewParamsList.size() - 1))
        .asps_log2_patch_packing_block_size(ceilLog2(m_blockSize));

    // Set AFPS parameters
    const auto &gi = m_outIvSequenceParams.vps.geometry_information(i);
    atlas.afps.afps_3d_pos_x_bit_count_minus1(gi.gi_geometry_3d_coordinates_bitdepth_minus1());
    atlas.afps.afps_3d_pos_y_bit_count_minus1(gi.gi_geometry_3d_coordinates_bitdepth_minus1());

    // Set ATGH parameters
    atlas.atgh.atgh_ref_atlas_frame_list_sps_flag(true);
    atlas.atgh.atgh_pos_min_z_quantizer(gi.gi_geometry_3d_coordinates_bitdepth_minus1() + 2);
    atlas.atgh.atgh_patch_size_x_info_quantizer(atlas.asps.asps_log2_patch_packing_block_size());
    atlas.atgh.atgh_patch_size_y_info_quantizer(atlas.asps.asps_log2_patch_packing_block_size());
  }
  if (m_maxEntities > 1) {
    m_packer->updateAggregatedEntityMasks(m_aggregatedEntityMask);
  }
  m_ivAccessUnitParams.patchParamsList =
      m_packer->pack(m_ivAccessUnitParams.atlasSizes(), aggregatedMask, m_isBasicView);

  // Atlas construction
  int frame = 0;
  for (const auto &views : m_viewBuffer) {
    MVD16Frame atlasList;

    for (uint8_t i = 0; i <= m_outIvSequenceParams.vps.vps_atlas_count_minus1(); ++i) {
      const auto frameWidth = m_outIvSequenceParams.vps.vps_frame_width(i);
      const auto frameHeight = m_outIvSequenceParams.vps.vps_frame_height(i);
      TextureDepth16Frame frame = {TextureFrame(frameWidth, frameHeight),
                                   Depth16Frame(frameWidth, frameHeight)};
      frame.texture.fillNeutral();
      frame.depth.fillZero();
      atlasList.push_back(move(frame));
    }

    for (const auto &patch : m_ivAccessUnitParams.patchParamsList) {
      const auto &view = views[patch.pduViewId()];
      if (m_maxEntities > 1) {
        // TODO(BK): refactor entitySeparator to accept a single view
        MVD16Frame tempViews;
        tempViews.push_back(view);
        const auto &entityViews = entitySeparator(tempViews, *patch.pduEntityId());
        writePatchInAtlas(patch, entityViews[0], atlasList, frame);
      } else {
        writePatchInAtlas(patch, view, atlasList, frame);
      }
    }
    m_atlasBuffer.push_back(move(atlasList));
    frame++;
  }

  return m_ivAccessUnitParams;
}

auto AtlasConstructor::popAtlas() -> MVD16Frame {
  MVD16Frame atlas = move(m_atlasBuffer.front());
  m_atlasBuffer.pop_front();
  return atlas;
}

auto AtlasConstructor::maxLumaSamplesPerFrame() const -> size_t { return m_maxLumaSamplesPerFrame; }

void AtlasConstructor::writePatchInAtlas(const PatchParams &patch,
                                         const TextureDepth16Frame &currentView, MVD16Frame &atlas,
                                         int frame) {
  auto &currentAtlas = atlas[patch.vuhAtlasId];

  auto &textureAtlasMap = currentAtlas.texture;
  auto &depthAtlasMap = currentAtlas.depth;

  const auto &textureViewMap = currentView.texture;
  const auto &depthViewMap = currentView.depth;
  int w = patch.pduViewSize().x();
  int h = patch.pduViewSize().y();
  int xM = patch.pduViewPos().x();
  int yM = patch.pduViewPos().y();

  const auto &inViewParams = m_inIvSequenceParams.viewParamsList[patch.pduViewId()];
  const auto &outViewParams = m_outIvSequenceParams.viewParamsList[patch.pduViewId()];

  for (int dyAligned = 0; dyAligned < h; dyAligned += m_blockSize) {
    for (int dxAligned = 0; dxAligned < w; dxAligned += m_blockSize) {

      bool isAggregatedMaskBlockNonEmpty = false;
      for (int dy = dyAligned; dy < dyAligned + m_blockSize; dy++) {
        if (dy + yM >= textureViewMap.getHeight() || dy + yM < 0) {
          continue;
        }
        for (int dx = dxAligned; dx < dxAligned + m_blockSize; dx++) {
          if (dx + xM >= textureViewMap.getWidth() || dx + xM < 0) {
            continue;
          }
          if (m_nonAggregatedMask[patch.pduViewId()](dy + yM, dx + xM)[frame]) {
            isAggregatedMaskBlockNonEmpty = true;
            break;
          }
        }
        if (isAggregatedMaskBlockNonEmpty) {
          break;
        }
      }

      for (int dy = dyAligned; dy < dyAligned + m_blockSize; dy++) {
        for (int dx = dxAligned; dx < dxAligned + m_blockSize; dx++) {

          Vec2i pView = {xM + dx, yM + dy};
          Vec2i pAtlas = patch.viewToAtlas(pView);

          if (pView.y() >= textureViewMap.getHeight() || pView.x() >= textureViewMap.getWidth() ||
              pAtlas.y() >= textureAtlasMap.getHeight() ||
              pAtlas.x() >= textureAtlasMap.getWidth() || pView.y() < 0 || pView.x() < 0 ||
              pAtlas.y() < 0 || pAtlas.x() < 0) {
            continue;
          }

          if (!isAggregatedMaskBlockNonEmpty) {
            depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;
            continue;
          }

          // Y
          textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) =
              textureViewMap.getPlane(0)(pView.y(), pView.x());
          // UV
          if ((pView.x() % 2) == 0 && (pView.y() % 2) == 0) {
            for (int p = 1; p < 3; ++p) {
              textureAtlasMap.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2) =
                  textureViewMap.getPlane(p)(pView.y() / 2, pView.x() / 2);
            }
          }

          // Depth
          auto depth = depthViewMap.getPlane(0)(pView.y(), pView.x());
          if (depth == 0 && !inViewParams.hasOccupancy && outViewParams.hasOccupancy &&
              m_maxEntities == 1) {
            depth = 1; // Avoid marking valid depth as invalid
          }
          depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = depth;
        }
      }
    }
  }
}
} // namespace TMIV::AtlasConstructor
