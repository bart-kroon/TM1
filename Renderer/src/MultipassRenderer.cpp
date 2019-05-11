/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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
#include <TMIV/Renderer/MultipassRenderer.h>
#include <iostream>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::Renderer {
MultipassRenderer::MultipassRenderer(const Common::Json &rootNode,
                                     const Common::Json &componentNode) {
  m_synthesizer = Factory<ISynthesizer>::getInstance().create(
      "Synthesizer", rootNode, componentNode);
  m_inpainter = Factory<IInpainter>::getInstance().create("Inpainter", rootNode,
                                                          componentNode);
  if (auto subnode = componentNode.optional("NumberOfPasses"))
    m_numberOfPasses = subnode.asInt();
  if (auto subnode = componentNode.optional("NumberOfViewsPerPass")) {
    if (subnode) {
      for (auto i = 0u; i != subnode.size(); i++)
        m_numberOfViewsPerPass.push_back(subnode.at(i).asInt());
    }
  }
  if (auto subnode = componentNode.optional("MergeConflict"))
    m_mergeConflict = subnode.asInt();
}

template <class _InIt1, class _InIt2, class _InIt3, class _InIt4, class _OutIt,
          class _Fn>
inline _OutIt my_transform(
    const _InIt1 _First1, const _InIt1 _Last1, const _InIt2 _First2,
    const _InIt3 _First3, const _InIt4 _First4, _OutIt _Dest,
    _Fn _Func) { // transform [_First1, _Last1) and [_First2, ...) with _Func
  _Adl_verify_range(_First1, _Last1);
  auto _UFirst1 = _Get_unwrapped(_First1);
  const auto _ULast1 = _Get_unwrapped(_Last1);
  const auto _Count = _Idl_distance<_InIt1>(_UFirst1, _ULast1);
  auto _UFirst2 = _Get_unwrapped_n(_First2, _Count);
  auto _UFirst3 = _Get_unwrapped_n(_First3, _Count);
  auto _UFirst4 = _Get_unwrapped_n(_First4, _Count);
  auto _UDest = _Get_unwrapped_n(_Dest, _Count);
  for (; _UFirst1 != _ULast1; ++_UFirst1, (void)++_UFirst2, (void)++_UFirst3,
                              (void)++_UFirst4, ++_UDest) {
    *_UDest = _Func(*_UFirst1, *_UFirst2, *_UFirst3, *_UFirst4);
  }

  _Seek_wrapped(_Dest, _UDest);
  return (_Dest);
}

uint16_t filterMergeDepth(uint16_t i, uint16_t j) {
  if (i >= j) // to enforce copying pixels if it is a foreground pixel only
    return i;
  else
    return j;
}

uint16_t filterDepthAfterMerge(uint16_t i, uint16_t j) {
  if (i > 0)
    return j;
  else
    return 0;
}

int mergeConflict;
uint16_t filterMergeTexture(uint16_t i, uint16_t j, uint16_t id, uint16_t jd) {
  if (i > 0) {
    if (id >= jd) // Checking depth
      return i;
    else // conflict
      switch (mergeConflict) {
      case 0:
        return 0; // return 0 values and let the inpainter fill them
      case 1:
        return i; // fill from the low-pass synthesis results which are in the
                  // background
      case 2:
        return j; // 2 fill from the high-pass synthesis results which are in
                  // the foreground
      }
  } else
    return j;
}

vector<unsigned int> selectedViewsPass, patchesViewId;
uint16_t filterMaps(uint16_t i) {
  if (i == unusedPatchId)
    return i;
  bool selectedPixel = false;
  for (auto selectedViewIndex = 0; selectedViewIndex < selectedViewsPass.size(); selectedViewIndex++) {
    if (patchesViewId[i] == selectedViewsPass[selectedViewIndex]) {
        selectedPixel = true;
    }
  }
  if (!selectedPixel)
    return unusedPatchId;
  else
    return i;
}

vector<size_t> sortViews(const Metadata::CameraParametersList &cameras,
                         const Metadata::CameraParameters &target) {
  float x_target = target.position[0];
  float y_target = target.position[1];
  float z_target = target.position[2];
  float yaw_target = target.rotation[0];
  float pitch_target = target.rotation[1];
  float roll_target = target.rotation[2];
  constexpr float radperdeg{0.01745329251994329576923690768489f};
  vector<float> distance, angle, angleWeight;
  for (auto id = 0u; id < cameras.size(); id++) {
    distance.push_back(sqrt(pow(cameras[id].position[0] - x_target, 2) +
                            pow(cameras[id].position[1] - y_target, 2) +
                            pow(cameras[id].position[2] - z_target, 2)));
    angle.push_back(1 / radperdeg *
                    acos(sin(cameras[id].rotation[1] * radperdeg) *
                             sin(pitch_target * radperdeg) +
                         cos(cameras[id].rotation[1] * radperdeg) *
                             cos(pitch_target * radperdeg) *
                             cos((cameras[id].rotation[0] - yaw_target) *
                                 radperdeg))); // Angle is in degree unit
    if (angle[id] > 180)
      angle[id] =
          angle[id] - 360; // to assure angle is ranging from -180 to 180 degree
    // Introduce AngleWeight as a simple triangle function (with value of 1 when
    // angle is 0 & value of 0 when angle is 180)
    if (angle[id] > 0.0)
      angleWeight.push_back(-1 / 180 * angle[id] + 1);
    else
      angleWeight.push_back(1 / 180 * angle[id] + 1);
  }
  // Find the sorted cameras indices
  vector<size_t> SortedCamerasId(cameras.size());
  iota(SortedCamerasId.begin(), SortedCamerasId.end(), 0); // initalization
  sort(SortedCamerasId.begin(), SortedCamerasId.end(),
       [&distance, &angleWeight](size_t i1, size_t i2) {
         return distance[i1] * angleWeight[i1] < distance[i2] * angleWeight[i2];
       });
  return SortedCamerasId;
}

Common::Texture444Depth10Frame
MultipassRenderer::renderFrame(const Common::MVD10Frame &atlas,
                               const Common::PatchIdMapList &maps,
                               const Metadata::AtlasParametersList &patches,
                               const Metadata::CameraParametersList &cameras,
                               const Metadata::CameraParameters &target) const {
  //////////////////
  // Initialization
  //////////////////
  int numberOfPasses = TMIV::Renderer::MultipassRenderer::m_numberOfPasses;
  vector<unsigned int> numberOfViewsPerPass = TMIV::Renderer::MultipassRenderer::m_numberOfViewsPerPass;
  mergeConflict = m_mergeConflict;

  if (numberOfPasses != numberOfViewsPerPass.size())
    cout << "WARNING: " << "Please check number of passes " << endl;

  Common::Texture444Depth10Frame viewport;
  Common::Texture444Depth10Frame *viewportPass;
  Common::PatchIdMapList *mapsPass;
  viewportPass = new Common::Texture444Depth10Frame[numberOfPasses];
  mapsPass = new Common::PatchIdMapList[numberOfPasses];

  for (auto j = 0; j < numberOfPasses; j++) {
    for (auto k = 0; k < atlas.size(); k++) {
      PatchIdMap patchMap(maps[k].getWidth(), maps[k].getHeight());
      std::fill(patchMap.getPlane(0).begin(), patchMap.getPlane(0).end(),
                unusedPatchId);
      mapsPass[j].push_back(patchMap);
    }
  } // initalize mapsPass by 0xFFFF

  for (auto patchId = 0; patchId < patches.size(); patchId++) {
    patchesViewId.push_back(patches[patchId].viewId);
  } // initialize patchesViewId

  ///////////////
  // Ordering views based on their distance & angle to target view
  ///////////////
  vector<size_t> SortedCamerasId(cameras.size());
  SortedCamerasId = sortViews(cameras, target);

  // Produce the individual pass synthesis results
  for (auto passId = 0; passId < numberOfPasses;
       passId++) // Loop over NumberOfPasses
  {
    // Find the selected views for a given pass
    selectedViewsPass.empty();
    for (auto id = 0u; id < cameras.size(); id++) {
      if (id < numberOfViewsPerPass[passId])
        selectedViewsPass.push_back(SortedCamerasId[id]);
    }

    /////////////////
    // Update the Occupancy Map to be used in the Pass
    /////////////////
    for (auto atlasId = 0; atlasId < maps.size(); atlasId++) {
      std::transform(maps[atlasId].getPlane(0).begin(),
                     maps[atlasId].getPlane(0).end(),
                     mapsPass[passId][atlasId].getPlane(0).begin(), filterMaps);
    }

    ////////////////
    // Synthesis per pass
    ////////////////
    viewportPass[passId] = m_synthesizer->renderFrame(atlas, mapsPass[passId],
                                                      patches, cameras, target);
  } // namespace TMIV::Renderer
  //////////////
  // Merging 
  //////////////
  if (numberOfPasses > 1) {
      Common::Texture444Depth10Frame mergedviewport = viewportPass[numberOfPasses - 1];
      for (auto passId = numberOfPasses - 1; passId > 0; passId--) {
          std::transform(viewportPass[passId - 1].second.getPlane(0).begin(),
              viewportPass[passId - 1].second.getPlane(0).end(),
              mergedviewport.second.getPlane(0).begin(),
              mergedviewport.second.getPlane(0).begin(),
              filterMergeDepth);
      
          for (auto i = 0; i < viewport.first.getNumberOfPlanes(); i++) {
              my_transform(viewportPass[passId - 1].first.getPlane(i).begin(),
                  viewportPass[passId - 1].first.getPlane(i).end(),
                  mergedviewport.first.getPlane(i).begin(),
                  viewportPass[passId - 1].second.getPlane(0).begin(),
                  mergedviewport.second.getPlane(0).begin(),
                  mergedviewport.first.getPlane(i).begin(),
                  filterMergeTexture);
      }
    }
    viewport = mergedviewport; // Final Merged

    std::transform(viewport.first.getPlane(0).begin(),
                   viewport.first.getPlane(0).end(),
                   viewport.second.getPlane(0).begin(),
                   viewport.second.getPlane(0).begin(), filterDepthAfterMerge);

  } else
    viewport = viewportPass[numberOfPasses - 1]; // Single Pass

  m_inpainter->inplaceInpaint(viewport, target);
  return viewport;
}

Common::Texture444Depth16Frame
MultipassRenderer::renderFrame(const Common::MVD16Frame &atlas,
                               const Metadata::CameraParametersList &cameras,
                               const Metadata::CameraParameters &target) const {
  auto viewport = m_synthesizer->renderFrame(atlas, cameras, target);
  m_inpainter->inplaceInpaint(viewport, target);
  return viewport;
}
} // namespace TMIV::Renderer
