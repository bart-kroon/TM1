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

#include <TMIV/Renderer/GroupBasedRenderer.h>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/Factory.h>

#include <cmath>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::Renderer {
GroupBasedRenderer::GroupBasedRenderer(const Json &rootNode, const Json &componentNode) {
  m_synthesizer =
      Factory<ISynthesizer>::getInstance().create("Synthesizer", rootNode, componentNode);
  m_inpainter = Factory<IInpainter>::getInstance().create("Inpainter", rootNode, componentNode);
}

template <class InIt1, class InIt2, class InIt3, class InIt4, class OutIt, class Fn>
void my_transform(InIt1 i1, InIt1 end1, InIt2 i2, InIt3 i3, InIt4 i4, OutIt dest, Fn Func) {
  for (; i1 != end1; ++i1, ++i2, ++i3, ++i4, ++dest) {
    *dest = Func(*i1, *i2, *i3, *i4);
  }
}

int mergeMode = 2;
auto filterGMergeDepth(uint16_t i, uint16_t j) -> uint16_t {
  if (i > 0) {
    if (i >= j) { // Checking depth
      return i;
    }
    // conflict
    switch (mergeMode) {
    case 0:
      return 0; // return 0 values and let the inpainter fill them
    case 1:
      return i; // fill from the low-pass synthesis results which are in the
                // background
    case 2:
      return j; // 2 fill from the high-pass synthesis results which are in
                // the foreground
    default:
      return 0;
    }
  }
  return j;
}

auto filterGMergeTexture(uint16_t i, uint16_t j, uint16_t id, uint16_t jd) -> uint16_t {
  if (i > 0) {
    if (id >= jd) { // Checking depth
      return i;
    }
    // conflict
    switch (mergeMode) {
    case 0:
      return 0; // return 0 values and let the inpainter fill them
    case 1:
      return i; // fill from the low-pass synthesis results which are in the
                // background
    case 2:
      return j; // 2 fill from the high-pass synthesis results which are in
                // the foreground
    default:
      return 0;
    }
  }
  return j;
}

vector<unsigned int> selectedGViewsPass, patchesGViewId;
auto filterGMaps(uint16_t i) -> uint16_t {
  if (i == unusedPatchId) {
    return i;
  }
  bool selectedPixel = false;
  for (auto selectedView : selectedGViewsPass) {
    if (patchesGViewId[i] == selectedView) {
      selectedPixel = true;
    }
  }
  if (!selectedPixel) {
    return unusedPatchId;
  }
  return i;
}

vector<uint8_t> groupIds;
vector<uint8_t> numberOfViewsPerPass;

auto sortGViews(const vector<vector<uint8_t>> &viewsPerGroup, 
				const ViewParamsVector &cameras,
                const ViewParams &target) -> vector<size_t> {
  float x_target = target.position[0];
  float y_target = target.position[1];
  float z_target = target.position[2];
  float yaw_target = target.rotation[0];
  float pitch_target = target.rotation[1];
  vector<vector<size_t>> sortedGCamerasId;
  vector<float> distanceG;
  size_t camSum = 0;
  for (int gIndex = 0; gIndex < viewsPerGroup.size(); gIndex++) {
    vector<uint8_t> views = viewsPerGroup[gIndex];
    vector<float> distance;
    vector<float> angle;
    vector<float> angleWeight;
    for (int id = 0; id < views.size(); id++) {
      distance.push_back(sqrt(square(cameras[views[id]].position[0] - x_target) +
                              square(cameras[views[id]].position[1] - y_target) +
                              square(cameras[views[id]].position[2] - z_target)));
      auto yaw_camera = cameras[views[id]].rotation[0];
      auto pitch_camera = cameras[views[id]].rotation[1];
      // Compute Angle between the camera and target in degree unit
      angle.push_back(1 / radperdeg *
                      acos(sin(pitch_camera * radperdeg) * sin(pitch_target * radperdeg) +
                           cos(pitch_camera * radperdeg) * cos(pitch_target * radperdeg) *
                               cos((yaw_camera - yaw_target) * radperdeg)));
      // to assure angle is ranging from -180 to 180 degree
      if (angle[id] > halfCycle) {
        angle[id] = angle[id] - fullCycle;
      }

      // Introduce AngleWeight as a simple triangle function (with value of 1 when
      // angle is 0 & value of 0 when angle is 180)
      if (angle[id] > 0.F) {
        angleWeight.push_back(-1.F / halfCycle * angle[id] + 1.F);
      } else {
        angleWeight.push_back(1.F / halfCycle * angle[id] + 1.F);
      }
    }
    // Find the sorted cameras indices per group
    vector<size_t> sortedCamerasId(views.size());
    iota(sortedCamerasId.begin(), sortedCamerasId.end(), 0); // initalization
    sort(sortedCamerasId.begin(), sortedCamerasId.end(),
         [&distance, &angleWeight](size_t i1, size_t i2) {
           if (angleWeight[i1] == angleWeight[i2]) {
             return distance[i1] < distance[i2];
           }
           return distance[i1] * (1.0 - angleWeight[i1]) < distance[i2] * (1.0 - angleWeight[i2]);
         });
    distanceG.push_back(distance[sortedCamerasId[0]]);
    for (int index = 0; index < sortedCamerasId.size(); index++)
      sortedCamerasId[index] = sortedCamerasId[index] + camSum;
    sortedGCamerasId.push_back(sortedCamerasId);
    camSum = camSum + sortedCamerasId.size();
  }
  // Find the nearest group
  vector<uint8_t> sortedDistanceId(distanceG.size());
  iota(sortedDistanceId.begin(), sortedDistanceId.end(), 0); // initalization
  sort(sortedDistanceId.begin(), sortedDistanceId.end(),
       [&distanceG](size_t i1, size_t i2) { return distanceG[i1] < distanceG[i2]; });

  vector<size_t> sortedGCamerasIdFinal;
  camSum = 0;
  for (auto dIndex : sortedDistanceId) {
    vector<size_t> views = sortedGCamerasId[dIndex];
    for (uint8_t vIndex = 0; vIndex < views.size(); vIndex++)
      sortedGCamerasIdFinal.push_back(views[vIndex]);
    camSum = camSum + views.size();
    numberOfViewsPerPass.push_back(camSum);
  }
  return sortedGCamerasIdFinal;
}

auto GroupBasedRenderer::renderFrame(const MVD10Frame &atlas, const PatchIdMapList &maps,
                                     const IvSequenceParams &ivSequenceParams,
                                     const IvAccessUnitParams &ivAccessUnitParams,
                                     const ViewParams &target) const -> Texture444Depth16Frame {
  //////////////////
  // Initialization
  //////////////////
  //const auto &patches = ivAccessUnitParams.atlasParamsList;
  // const auto &groupId = inAccessUnitParams.atlasParamsList->groupIds;
  if (ivSequenceParams.numGroups>1)
	groupIds = {0, 0, 1, 1, 2, 2};// Need to be updated from the atlas metadata ////////////////////////////////////////

  if (groupIds.size() == 0)
    groupIds.assign(atlas.size(), 0);
  int numberOfPasses = ivSequenceParams.numGroups;
  if (ivSequenceParams.depthLowQualityFlag)
    mergeMode = 1;
  else
    mergeMode = 2;

  Texture444Depth16Frame viewport;
  vector<Texture444Depth16Frame> viewportPass(numberOfPasses);
  vector<PatchIdMapList> mapsPass(numberOfPasses);

  for (auto &pass : mapsPass) {
    for (size_t k = 0; k < atlas.size(); ++k) {
      PatchIdMap patchMap(maps[k].getWidth(), maps[k].getHeight());
      fill(patchMap.getPlane(0).begin(), patchMap.getPlane(0).end(), unusedPatchId);
      pass.push_back(patchMap);
    }
  } // initalize mapsPass by 0xFFFF

  assert(ivAccessUnitParams.atlasParamsList);
  const auto &atlasParamsList = *ivAccessUnitParams.atlasParamsList;

  for (const auto &patch : atlasParamsList) {
    patchesGViewId.push_back(patch.viewId);
  } // initialize patchesViewId

  // Find views per group
  vector<vector<uint8_t>> viewsPerGroup;// {{2, 3, 1}, {0, 4, 5}, {6, 7, 8, 9}};
  for (int gIndex = 0; gIndex < ivSequenceParams.numGroups; gIndex++) {
    vector<uint8_t> atlasIds;
    for (int aIndex = 0; aIndex < groupIds.size(); aIndex++) {
      if (groupIds[aIndex] == gIndex)
        atlasIds.push_back(aIndex);
    }
    vector<uint8_t> views;
    for (int pIndex = 0; pIndex < atlasParamsList.size(); pIndex++) {
      for (int aIndex = 0; aIndex < atlasIds.size(); aIndex++) {
        if (atlasParamsList[pIndex].atlasId == atlasIds[aIndex]) {
          for (int vIndex = 0; vIndex < views.size(); vIndex++) {
            if (views[vIndex] == atlasParamsList[pIndex].viewId)
              goto NextPatch; // Already captured value, skip;
          }
          views.push_back(atlasParamsList[pIndex].viewId);
        NextPatch:
          0;
        }
      }
    }
    viewsPerGroup.push_back(views);
  }
  ///////////////
  // Ordering views based on their distance & angle to target view
  ///////////////
  vector<size_t> sortedCamerasId(ivSequenceParams.viewParamsList.size());
  sortedCamerasId = sortGViews(viewsPerGroup,ivSequenceParams.viewParamsList, target);
  if (numberOfViewsPerPass.size() == 0) // in case of numberOfGroups = 1
    numberOfViewsPerPass.push_back(ivSequenceParams.viewParamsList.size());

  // Produce the individual pass synthesis results
  for (int passId = 0; passId < numberOfPasses; passId++) // Loop over NumberOfPasses
  {
    // Find the selected views for a given pass
    selectedGViewsPass.clear();
    for (size_t id = 0; id < ivSequenceParams.viewParamsList.size(); ++id) {
      if (id < numberOfViewsPerPass[passId]) {
        selectedGViewsPass.push_back(static_cast<unsigned int>(sortedCamerasId[id]));
      }
    }

	printf("Selected Optimized Views in Pass %d : ", passId);
    for (auto i = 0; i < selectedGViewsPass.size(); i++) {
      printf("o%d, ", selectedGViewsPass[i]);
    }
    printf("\n");

    /////////////////
    // Update the Occupancy Map to be used in the Pass
    /////////////////
    for (size_t atlasId = 0; atlasId < maps.size(); ++atlasId) {
      transform(maps[atlasId].getPlane(0).begin(), maps[atlasId].getPlane(0).end(),
                mapsPass[passId][atlasId].getPlane(0).begin(), filterGMaps);
    }

    ////////////////
    // Synthesis per pass
    ////////////////
    viewportPass[passId] = m_synthesizer->renderFrame(atlas, mapsPass[passId], ivSequenceParams,
                                                      ivAccessUnitParams, target);
  }

  //////////////
  // Merging
  //////////////
  if (numberOfPasses > 1) {
    Texture444Depth16Frame mergedviewport = viewportPass[numberOfPasses - 1];
    for (auto passId = numberOfPasses - 1; passId > 0; passId--) {
      transform(viewportPass[passId - 1].second.getPlane(0).begin(),
                viewportPass[passId - 1].second.getPlane(0).end(),
                mergedviewport.second.getPlane(0).begin(),
                mergedviewport.second.getPlane(0).begin(), filterGMergeDepth);

      for (auto i = 0; i < viewport.first.getNumberOfPlanes(); ++i) {
        my_transform(viewportPass[passId - 1].first.getPlane(i).begin(),
                     viewportPass[passId - 1].first.getPlane(i).end(),
                     mergedviewport.first.getPlane(i).begin(),
                     viewportPass[passId - 1].second.getPlane(0).begin(),
                     mergedviewport.second.getPlane(0).begin(),
                     mergedviewport.first.getPlane(i).begin(), filterGMergeTexture);
      }
    }
    viewport = mergedviewport; // Final Merged

  } else {
    viewport = viewportPass[numberOfPasses - 1]; // Single Pass
  }

  m_inpainter->inplaceInpaint(viewport, target);
  return viewport;
}
} // namespace TMIV::Renderer
