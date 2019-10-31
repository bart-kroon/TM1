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

#include <TMIV/Encoder/GroupBasedEncoder.h>

#include <algorithm>
#include <cassert>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::Encoder {
GroupBasedEncoder::GroupBasedEncoder(const Json &rootNode, const Json &componentNode) {
  const auto numGroups_ = size_t(rootNode.require("numGroups").asInt());

  while (m_encoders.size() < numGroups_) {
    m_encoders.emplace_back(rootNode, componentNode);
  }
}

auto GroupBasedEncoder::prepareSequence(IvSequenceParams ivSequenceParams)
    -> const IvSequenceParams & {
  m_grouping = groupSelector(ivSequenceParams);

  auto perGroupIvSequenceParams = vector<const IvSequenceParams *>(numGroups(), nullptr);

  for (size_t groupId = 0; groupId != numGroups(); ++groupId) {
    perGroupIvSequenceParams[groupId] =
        &m_encoders[groupId].prepareSequence(splitSequenceParams(groupId, ivSequenceParams));
  }

  return mergeSequenceParams(perGroupIvSequenceParams);
}

void GroupBasedEncoder::prepareAccessUnit(IvAccessUnitParams ivAccessUnitParams) {
  for (std::size_t groupId = 0; groupId != numGroups(); ++groupId) {
    m_encoders[groupId].prepareAccessUnit(ivAccessUnitParams);
  }
}

void GroupBasedEncoder::pushFrame(MVD16Frame views) {
  for (std::size_t groupId = 0; groupId != numGroups(); ++groupId) {
    cout << "Processing G" << groupId << " : \n";
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

auto GroupBasedEncoder::groupSelector(const Metadata::IvSequenceParams &ivSequenceParams)
    -> Grouping {
  auto grouping = Grouping{};

  const auto &cameras = ivSequenceParams.viewParamsList;
  const auto numGroups = ivSequenceParams.numGroups;

  // Compute axial ranges and find the dominant one
  vector<float> Tx, Ty, Tz;
  for (int camIndex = 0; camIndex < cameras.size(); camIndex++) {
    Tx.push_back(cameras[camIndex].position[0]);
    Ty.push_back(cameras[camIndex].position[1]);
    Tz.push_back(cameras[camIndex].position[2]);
  }

  const float xMax = *std::max_element(Tx.begin(), Tx.end());
  const float xMin = *std::min_element(Tx.begin(), Tx.end());
  const float yMax = *std::max_element(Ty.begin(), Ty.end());
  const float yMin = *std::min_element(Ty.begin(), Ty.end());
  const float zMax = *std::max_element(Tz.begin(), Tz.end());
  const float zMin = *std::min_element(Tz.begin(), Tz.end());

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
  auto viewsPool = vector<Metadata::ViewParams>{};
  auto viewsLabels = vector<uint8_t>{};
  auto viewsInGroup = vector<uint8_t>{};
  auto numViewsPerGroup = vector<int>{};

  for (size_t camIndex = 0; camIndex < cameras.size(); camIndex++) {
    viewsPool.push_back(cameras[camIndex]);
    viewsLabels.push_back(uint8_t(camIndex));
  }

  for (unsigned gIndex = 0; gIndex < numGroups; gIndex++) {
    viewsInGroup.clear();
    Metadata::ViewParamsList camerasInGroup, camerasOutGroup;
    if (gIndex < numGroups - 1) {
      numViewsPerGroup.push_back((int)std::floor(cameras.size() / numGroups));
      std::int64_t maxElementIndex;

      if (dominantAxis == 0) {
        maxElementIndex = std::max_element(Tx.begin(), Tx.end()) - Tx.begin();
      } else if (dominantAxis == 1) {
        maxElementIndex = std::max_element(Ty.begin(), Ty.end()) - Ty.begin();
      } else {
        maxElementIndex = std::max_element(Tz.begin(), Tz.end()) - Tz.begin();
      }

      const auto T0 = Vec3f{Tx[maxElementIndex], Ty[maxElementIndex], Tz[maxElementIndex]};
      vector<float> distance;
      for (size_t id = 0; id < viewsPool.size(); ++id)
        distance.push_back(norm(viewsPool[id].position - T0));

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
        Tx.push_back(viewsPool[sortedCamerasId[camIndex]].position[0]);
        Ty.push_back(viewsPool[sortedCamerasId[camIndex]].position[1]);
        Tz.push_back(viewsPool[sortedCamerasId[camIndex]].position[2]);
      }

      cout << "Views (0-based) Selected for G" << gIndex << " : ";
      for (size_t i = 0; i < camerasInGroup.size(); i++) {
        cout << "v" << unsigned(viewsLabels[sortedCamerasId[i]]) << ", ";
        viewsInGroup.push_back(viewsLabels[sortedCamerasId[i]]);
      }
      cout << "\n";

      vector<uint8_t> viewLabelsTemp;
      for (auto i = camerasInGroup.size(); i < viewsLabels.size(); i++)
        viewLabelsTemp.push_back(viewsLabels[sortedCamerasId[i]]);
      viewsLabels.assign(viewLabelsTemp.begin(), viewLabelsTemp.end());

      viewsPool.clear();
      for (size_t camIndex = 0; camIndex < camerasOutGroup.size(); camIndex++)
        viewsPool.push_back(camerasOutGroup[camIndex]);
    } else {
      camerasInGroup.clear();
      numViewsPerGroup.push_back(
          int((cameras.size() - (numGroups - 1) * std::floor(cameras.size() / numGroups))));
      for (size_t camIndex = 0; camIndex < viewsPool.size(); camIndex++) {
        camerasInGroup.push_back(viewsPool[camIndex]);
      }

      cout << "Views (0-based) Selected for G" << gIndex << " : ";
      for (auto i = 0; i < camerasInGroup.size(); i++) {
        cout << "v" << unsigned(viewsLabels[i]) << ", ";
        viewsInGroup.push_back(viewsLabels[i]);
      }
      cout << "\n";
    }
    for (size_t viewId = 0; viewId < viewsInGroup.size(); ++viewId) {
      grouping.emplace_back(gIndex, viewsInGroup[viewId]);
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
  for (const auto [groupId_, viewId] : m_grouping) {
    if (groupId_ == groupId) {
      result.viewParamsList.push_back(ivSequenceParams.viewParamsList[viewId]);
    }
  }

  return result;
}

auto GroupBasedEncoder::splitViews(size_t groupId, MVD16Frame &views) const -> MVD16Frame {
  auto result = MVD16Frame{};

  // Only include the views that are part of this group
  for (const auto [groupId_, viewId] : m_grouping) {
    if (groupId_ == groupId) {
      result.push_back(views[viewId]);
    }
  }

  return result;
}

auto GroupBasedEncoder::mergeSequenceParams(
    const std::vector<const Metadata::IvSequenceParams *> &perGroupParams)
    -> const Metadata::IvSequenceParams & {
  // Independent metadata should work automatically. Just assume it is the same across groups.
  m_ivSequenceParams = *perGroupParams.front();
  m_ivSequenceParams.viewParamsList.clear();

  // Copy view parameters in group order
  for (auto groupParams : perGroupParams) {
    copy(begin(groupParams->viewParamsList), end(groupParams->viewParamsList),
         back_inserter(m_ivSequenceParams.viewParamsList));
  }

  // Keep around for mergeAccessUnitParams
  m_perGroupSequenceParams = perGroupParams;

  return m_ivSequenceParams;
}

auto GroupBasedEncoder::mergeAccessUnitParams(
    const std::vector<const Metadata::IvAccessUnitParams *> &perGroupParams)
    -> const Metadata::IvAccessUnitParams & {

  // Independent metadata should work automatically. Just assume it is the same across groups.
  m_ivAccessUnitParams = *perGroupParams.front();
  auto &atlasParamsList = *m_ivAccessUnitParams.atlasParamsList;
  atlasParamsList.clear();
  atlasParamsList.groupIds = vector<unsigned>{};
  atlasParamsList.atlasSizes.clear();

  size_t firstAtlasId = 0;
  size_t firstViewId = 0;

  for (size_t groupId = 0; groupId < numGroups(); ++groupId) {
    const auto &groupParams = *perGroupParams[groupId];

    // Copy patches in group order
    for (auto patch : *groupParams.atlasParamsList) {
      patch.atlasId += uint16_t(firstAtlasId);
      patch.viewId += uint16_t(firstViewId);
      atlasParamsList.push_back(patch);
    }

    // Copy atlas sizes in group order
    copy(begin(groupParams.atlasParamsList->atlasSizes),
         end(groupParams.atlasParamsList->atlasSizes), back_inserter(atlasParamsList.atlasSizes));

    // Assign group ID's
    while (atlasParamsList.groupIds->size() < atlasParamsList.atlasSizes.size()) {
      atlasParamsList.groupIds->push_back(unsigned(groupId));
    }

    // Renumber atlases and views
    firstAtlasId += groupParams.atlasParamsList->atlasSizes.size();
    firstViewId += m_perGroupSequenceParams[groupId]->viewParamsList.size();
  }

  return m_ivAccessUnitParams;
}

} // namespace TMIV::Encoder
