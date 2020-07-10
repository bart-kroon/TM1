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

#include <TMIV/Encoder/GroupBasedEncoder.h>

#include <algorithm>
#include <cassert>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::Encoder {
GroupBasedEncoder::GroupBasedEncoder(const Json &rootNode, const Json &componentNode) {
  const auto numGroups_ = size_t(rootNode.require("numGroups").asInt());

  while (m_encoders.size() < numGroups_) {
    m_encoders.emplace_back(rootNode, componentNode);
  }
}

auto GroupBasedEncoder::prepareSequence(IvSequenceParams ivSequenceParams)
    -> const IvSequenceParams & {
  m_grouping = sourceSplitter(ivSequenceParams);

  auto perGroupIvSequenceParams = vector<const IvSequenceParams *>(numGroups(), nullptr);

  for (size_t groupId = 0; groupId != numGroups(); ++groupId) {
    perGroupIvSequenceParams[groupId] =
        &m_encoders[groupId].prepareSequence(splitSequenceParams(groupId, ivSequenceParams));
  }

  return mergeSequenceParams(perGroupIvSequenceParams);
}

void GroupBasedEncoder::prepareAccessUnit() {
  for (size_t groupId = 0; groupId != numGroups(); ++groupId) {
    m_encoders[groupId].prepareAccessUnit();
  }
}

void GroupBasedEncoder::pushFrame(MVD16Frame views) {
  for (size_t groupId = 0; groupId != numGroups(); ++groupId) {
    cout << "Processing group " << groupId << ":\n";
    m_encoders[groupId].pushFrame(splitViews(groupId, views));
  }
}

auto GroupBasedEncoder::completeAccessUnit() -> const IvAccessUnitParams & {
  auto perGroupIvAccessUnitParams = vector<const IvAccessUnitParams *>(numGroups(), nullptr);

  for (size_t groupId = 0; groupId != numGroups(); ++groupId) {
    perGroupIvAccessUnitParams[groupId] = &m_encoders[groupId].completeAccessUnit();
  }

  return mergeAccessUnitParams(perGroupIvAccessUnitParams);
}

auto GroupBasedEncoder::popAtlas() -> MVD10Frame {
  auto result = MVD10Frame{};

  for (auto &encoder : m_encoders) {
    for (auto &atlas : encoder.popAtlas()) {
      result.push_back(move(atlas));
    }
  }

  return result;
}

auto GroupBasedEncoder::maxLumaSamplesPerFrame() const -> size_t {
  return accumulate(m_encoders.begin(), m_encoders.end(), size_t{},
                    [](size_t sum, const auto &x) { return sum + x.maxLumaSamplesPerFrame(); });
}

auto GroupBasedEncoder::sourceSplitter(const IvSequenceParams &ivSequenceParams) -> Grouping {
  auto grouping = Grouping{};

  const auto &viewParamsList = ivSequenceParams.viewParamsList;
  const auto numGroups = ivSequenceParams.vme().vme_num_groups_minus1() + 1;

  // Compute axial ranges and find the dominant one
  auto Tx = vector<float>{};
  auto Ty = vector<float>{};
  auto Tz = vector<float>{};
  for (size_t camIndex = 0; camIndex < viewParamsList.size(); camIndex++) {
    Tx.push_back(viewParamsList[camIndex].ce.ce_view_pos_x());
    Ty.push_back(viewParamsList[camIndex].ce.ce_view_pos_y());
    Tz.push_back(viewParamsList[camIndex].ce.ce_view_pos_z());
  }

  const float xMax = *max_element(Tx.begin(), Tx.end());
  const float xMin = *min_element(Tx.begin(), Tx.end());
  const float yMax = *max_element(Ty.begin(), Ty.end());
  const float yMin = *min_element(Ty.begin(), Ty.end());
  const float zMax = *max_element(Tz.begin(), Tz.end());
  const float zMin = *min_element(Tz.begin(), Tz.end());

  const float xRange = xMax - xMin;
  const float yRange = yMax - yMin;
  const float zRange = zMax - zMin;

  int dominantAxis = 0;
  if (zRange >= xRange && zRange >= yRange) {
    dominantAxis = 2;
  } else if (yRange >= xRange && yRange >= zRange) {
    dominantAxis = 1;
  }

  // Select views per group
  auto viewsPool = vector<ViewParams>{};
  auto viewsLabels = vector<uint8_t>{};
  auto viewsInGroup = vector<uint8_t>{};
  auto numViewsPerGroup = vector<int>{};

  for (size_t camIndex = 0; camIndex < viewParamsList.size(); camIndex++) {
    viewsPool.push_back(viewParamsList[camIndex]);
    viewsLabels.push_back(uint8_t(camIndex));
  }

  for (unsigned gIndex = 0; gIndex < numGroups; gIndex++) {
    viewsInGroup.clear();
    auto camerasInGroup = ViewParamsList{};
    auto camerasOutGroup = ViewParamsList{};
    if (gIndex < numGroups - 1) {
      numViewsPerGroup.push_back(int(floor(viewParamsList.size() / numGroups)));
      int64_t maxElementIndex = 0;

      if (dominantAxis == 0) {
        maxElementIndex = max_element(Tx.begin(), Tx.end()) - Tx.begin();
      } else if (dominantAxis == 1) {
        maxElementIndex = max_element(Ty.begin(), Ty.end()) - Ty.begin();
      } else {
        maxElementIndex = max_element(Tz.begin(), Tz.end()) - Tz.begin();
      }

      const auto T0 = Vec3f{Tx[maxElementIndex], Ty[maxElementIndex], Tz[maxElementIndex]};
      auto distance = vector<float>();
      distance.reserve(viewsPool.size());
      for (const auto &viewParams : viewsPool) {
        distance.push_back(norm(viewParams.ce.position() - T0));
      }

      // ascending order
      vector<size_t> sortedCamerasId(viewsPool.size());
      iota(sortedCamerasId.begin(), sortedCamerasId.end(), 0); // initalization
      sort(sortedCamerasId.begin(), sortedCamerasId.end(),
           [&distance](size_t i1, size_t i2) { return distance[i1] < distance[i2]; });
      for (int camIndex = 0; camIndex < numViewsPerGroup[gIndex]; camIndex++) {
        camerasInGroup.push_back(viewsPool[sortedCamerasId[camIndex]]);
      }

      // update the viewsPool
      Tx.clear();
      Ty.clear();
      Tz.clear();
      camerasOutGroup.clear();
      for (size_t camIndex = numViewsPerGroup[gIndex]; camIndex < viewsPool.size(); camIndex++) {
        camerasOutGroup.push_back(viewsPool[sortedCamerasId[camIndex]]);
        Tx.push_back(viewsPool[sortedCamerasId[camIndex]].ce.ce_view_pos_x());
        Ty.push_back(viewsPool[sortedCamerasId[camIndex]].ce.ce_view_pos_y());
        Tz.push_back(viewsPool[sortedCamerasId[camIndex]].ce.ce_view_pos_z());
      }

      cout << "Views selected for group " << gIndex << ": ";
      const auto *sep = "";
      for (size_t i = 0; i < camerasInGroup.size(); i++) {
        cout << sep << unsigned(viewsLabels[sortedCamerasId[i]]);
        viewsInGroup.push_back(viewsLabels[sortedCamerasId[i]]);
        sep = ", ";
      }
      cout << '\n';

      auto viewLabelsTemp = vector<uint8_t>{};
      for (size_t i = camerasInGroup.size(); i < viewsLabels.size(); i++) {
        viewLabelsTemp.push_back(viewsLabels[sortedCamerasId[i]]);
      }
      viewsLabels.assign(viewLabelsTemp.begin(), viewLabelsTemp.end());

      viewsPool = camerasOutGroup;
    } else {
      numViewsPerGroup.push_back(int(
          (viewParamsList.size() - (numGroups - 1) * floor(viewParamsList.size() / numGroups))));

      camerasInGroup.clear();
      copy(cbegin(viewsPool), cend(viewsPool), back_inserter(camerasInGroup));

      cout << "Views selected for group " << gIndex << ": ";
      const auto *sep = "";
      for (size_t i = 0; i < camerasInGroup.size(); i++) {
        cout << sep << unsigned(viewsLabels[i]);
        viewsInGroup.push_back(viewsLabels[i]);
        sep = ", ";
      }
      cout << '\n';
    }
    for (const auto viewInGroup : viewsInGroup) {
      grouping.emplace_back(gIndex, viewInGroup);
    }
  }
  return grouping;
}

auto GroupBasedEncoder::splitSequenceParams(size_t groupId,
                                            const IvSequenceParams &ivSequenceParams) const
    -> IvSequenceParams {
  // Independent metadata should work automatically. Just copy all metadata to all groups.
  auto result = ivSequenceParams;
  result.viewParamsList.clear();

  // Only include the views that are part of this group
  for (const auto &[groupId_, viewId] : m_grouping) {
    if (groupId_ == groupId) {
      result.viewParamsList.push_back(ivSequenceParams.viewParamsList[viewId]);
    }
  }

  return result;
}

auto GroupBasedEncoder::splitViews(size_t groupId, MVD16Frame &views) const -> MVD16Frame {
  auto result = MVD16Frame{};

  // Only include the views that are part of this group
  for (const auto &[groupId_, viewId] : m_grouping) {
    if (groupId_ == groupId) {
      result.push_back(views[viewId]);
    }
  }

  return result;
}

auto GroupBasedEncoder::mergeSequenceParams(const vector<const IvSequenceParams *> &perGroupParams)
    -> const IvSequenceParams & {
  // Start with first group
  m_ivSequenceParams = *perGroupParams.front();
  assert(m_ivSequenceParams.vme().vme_num_groups_minus1() + 1 == perGroupParams.size());

  // Merge V3C parameter sets
  vector<const V3cParameterSet *> vps(perGroupParams.size());
  transform(begin(perGroupParams), end(perGroupParams), begin(vps),
            [](const auto &ivs) { return &ivs->vps; });
  m_ivSequenceParams.vps = merge(vps);

  // For each other group
  for (auto ivs = begin(perGroupParams) + 1; ivs != end(perGroupParams); ++ivs) {
    // Merge view parameters
    transform(begin((*ivs)->viewParamsList), end((*ivs)->viewParamsList),
              back_inserter(m_ivSequenceParams.viewParamsList),
              [viewIdOffset = uint16_t(m_ivSequenceParams.viewParamsList.size())](ViewParams vp) {
                // Merging pruning graphs
                if (vp.pp && !vp.pp->pp_is_root_flag()) {
                  for (uint16_t i = 0; i <= vp.pp->pp_num_parent_minus1(); ++i) {
                    vp.pp->pp_parent_id(i, vp.pp->pp_parent_id(i) + viewIdOffset);
                  }
                }
                return vp;
              });

    // Merge viewing space
    assert(m_ivSequenceParams.viewingSpace == (*ivs)->viewingSpace);
  }

  // Merge MVPL across groups
  m_ivSequenceParams.mvpl.mvp_num_views_minus1(
      uint16_t(m_ivSequenceParams.viewParamsList.size() - 1));
  int aIndex = 0, vIndex = 0, sumViewsInGroups = 0;
  for (unsigned g = 0; g <= m_ivSequenceParams.vme().vme_num_groups_minus1(); g++) {
    for (uint8_t a = 0; a <= perGroupParams[g]->vps.vps_atlas_count_minus1(); a++) {
      for (uint16_t v = 0; v <= perGroupParams[g]->mvpl.mvp_num_views_minus1(); v++) {
        vIndex = v + sumViewsInGroups;
        m_ivSequenceParams.mvpl.mvp_view_enabled_in_atlas_flag(
            aIndex, vIndex, perGroupParams[g]->mvpl.mvp_view_enabled_in_atlas_flag(a, v));
        if (perGroupParams[g]->mvpl.mvp_view_enabled_in_atlas_flag(a, v)) {
          m_ivSequenceParams.mvpl.mvp_view_complete_in_atlas_flag(
              aIndex, vIndex, perGroupParams[g]->mvpl.mvp_view_complete_in_atlas_flag(a, v));
        }
      }
      aIndex++;
    }
    sumViewsInGroups = sumViewsInGroups + perGroupParams[g]->mvpl.mvp_num_views_minus1() + 1;
  }
  m_ivSequenceParams.mvpl.mvp_explicit_view_id_flag(false);
  for (unsigned g = 0; g <= m_ivSequenceParams.vme().vme_num_groups_minus1(); g++) {
    if (perGroupParams[g]->mvpl.mvp_explicit_view_id_flag()) {
      m_ivSequenceParams.mvpl.mvp_explicit_view_id_flag(true);
    }
  }
  if (m_ivSequenceParams.mvpl.mvp_explicit_view_id_flag()) {
    int vIndex = 0;
    sumViewsInGroups = 0;
    for (unsigned g = 0; g <= m_ivSequenceParams.vme().vme_num_groups_minus1(); g++) {
      for (uint16_t v = 0; v <= perGroupParams[g]->mvpl.mvp_num_views_minus1(); v++) {
        m_ivSequenceParams.mvpl.mvp_view_id(vIndex++, perGroupParams[g]->mvpl.mvp_view_id(v) +
                                                            (uint16_t)sumViewsInGroups);
      }
      sumViewsInGroups = sumViewsInGroups + perGroupParams[g]->mvpl.mvp_num_views_minus1() + 1;
    }
  }

  // Keep around for mergeAccessUnitParams
  m_perGroupSequenceParams = perGroupParams;

  return m_ivSequenceParams;
}

auto GroupBasedEncoder::mergeAccessUnitParams(
    const vector<const IvAccessUnitParams *> &perGroupParams) -> const IvAccessUnitParams & {
  // No state at this level
  m_ivAccessUnitParams = {};

  // Concatenate atlas access unit parameters
  for (size_t groupId = 0; groupId < perGroupParams.size(); ++groupId) {
    for (const auto &atlas : perGroupParams[groupId]->atlas) {
      m_ivAccessUnitParams.atlas.push_back(atlas);

      // Set group ID
      m_ivAccessUnitParams.atlas.back().asme().asme_group_id(unsigned(groupId));
    }
  }

  // Modify bit depth of pdu_projection_id
  for (auto &atlas : m_ivAccessUnitParams.atlas) {
    atlas.asps.asps_extended_projection_enabled_flag(true).asps_max_number_projections_minus1(
        uint16_t(m_ivSequenceParams.viewParamsList.size() - 1));
  }

  // Renumber atlas and view ID's
  uint16_t atlasIdOffset = 0;
  uint16_t viewIdOffset = 0;

  for (size_t groupId = 0; groupId < perGroupParams.size(); ++groupId) {
    // Copy patches in group order
    for (const auto &patch : perGroupParams[groupId]->patchParamsList) {
      m_ivAccessUnitParams.patchParamsList.push_back(patch);
      m_ivAccessUnitParams.patchParamsList.back().vuhAtlasId += atlasIdOffset;
      m_ivAccessUnitParams.patchParamsList.back().pduViewId(patch.pduViewId() + viewIdOffset);
    }

    // Renumber atlases and views
    atlasIdOffset += uint16_t(perGroupParams[groupId]->atlas.size());
    viewIdOffset += uint16_t(m_perGroupSequenceParams[groupId]->viewParamsList.size());
  }

  return m_ivAccessUnitParams;
}

} // namespace TMIV::Encoder
