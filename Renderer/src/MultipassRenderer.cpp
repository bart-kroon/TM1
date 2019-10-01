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

#include <TMIV/Renderer/MultipassRenderer.h>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/Factory.h>
#include <cmath>
#include <iostream>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::Renderer {
MultipassRenderer::MultipassRenderer(const Json &rootNode, const Json &componentNode) {
  m_synthesizer =
      Factory<ISynthesizer>::getInstance().create("Synthesizer", rootNode, componentNode);
  m_inpainter = Factory<IInpainter>::getInstance().create("Inpainter", rootNode, componentNode);
  m_numberOfPasses = componentNode.require("NumberOfPasses").asInt();
  auto subnode = componentNode.require("NumberOfViewsPerPass");
  for (size_t i = 0; i != subnode.size(); ++i) {
    m_numberOfViewsPerPass.push_back(subnode.at(i).asInt());
  }
  if (auto subnode = componentNode.optional("MergeConflict")) {
    m_mergeConflict = subnode.asInt();
  }
}

template <class InIt1, class InIt2, class InIt3, class InIt4, class OutIt, class Fn>
void my_transform(InIt1 i1, InIt1 end1, InIt2 i2, InIt3 i3, InIt4 i4, OutIt dest, Fn Func) {
  for (; i1 != end1; ++i1, ++i2, ++i3, ++i4, ++dest) {
    *dest = Func(*i1, *i2, *i3, *i4);
  }
}

int mergeConflict;
uint16_t filterMergeDepth(uint16_t i, uint16_t j) {
  if (i > 0) {
    if (i >= j) { // Checking depth
      return i;
    }
    // conflict
    switch (mergeConflict) {
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

uint16_t filterMergeTexture(uint16_t i, uint16_t j, uint16_t id, uint16_t jd) {
  if (i > 0) {
    if (id >= jd) { // Checking depth
      return i;
    }
    // conflict
    switch (mergeConflict) {
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

vector<unsigned int> selectedViewsPass, patchesViewId;
uint16_t filterMaps(uint16_t i) {
  if (i == unusedPatchId) {
    return i;
  }
  bool selectedPixel = false;
  for (auto selectedView : selectedViewsPass) {
    if (patchesViewId[i] == selectedView) {
      selectedPixel = true;
    }
  }
  if (!selectedPixel) {
    return unusedPatchId;
  }
  return i;
}

vector<size_t> sortViews(const Metadata::CameraParametersList &cameras,
                         const Metadata::CameraParameters &target) {
  float x_target = target.position[0];
  float y_target = target.position[1];
  float z_target = target.position[2];
  float yaw_target = target.rotation[0];
  float pitch_target = target.rotation[1];
  vector<float> distance;
  vector<float> angle;
  vector<float> angleWeight;
  for (size_t id = 0; id < cameras.size(); ++id) {
    distance.push_back(sqrt(square(cameras[id].position[0] - x_target) +
                            square(cameras[id].position[1] - y_target) +
                            square(cameras[id].position[2] - z_target)));
    auto yaw_camera = cameras[id].rotation[0];
    auto pitch_camera = cameras[id].rotation[1];
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

  // Find the sorted cameras indices
  vector<size_t> sortedCamerasId(cameras.size());
  iota(sortedCamerasId.begin(), sortedCamerasId.end(), 0); // initalization
  sort(sortedCamerasId.begin(), sortedCamerasId.end(),
       [&distance, &angleWeight](size_t i1, size_t i2) {
         if (angleWeight[i1] == angleWeight[i2]) {
           return distance[i1] < distance[i2];
         }
         return distance[i1] * (1.0 - angleWeight[i1]) < distance[i2] * (1.0 - angleWeight[i2]);
       });
  return sortedCamerasId;
}

Texture444Depth16Frame
MultipassRenderer::renderFrame(const MVD10Frame &atlas, const PatchIdMapList &maps,
                               const Metadata::AtlasParametersList &patches,
                               const Metadata::CameraParametersList &cameras,
                               const Metadata::CameraParameters &target) const {
  //////////////////
  // Initialization
  //////////////////
  int numberOfPasses = TMIV::Renderer::MultipassRenderer::m_numberOfPasses;
  vector<unsigned int> numberOfViewsPerPass =
      TMIV::Renderer::MultipassRenderer::m_numberOfViewsPerPass;
  mergeConflict = m_mergeConflict;

  if (numberOfPasses != int(numberOfViewsPerPass.size())) {
    cout << "WARNING: "
         << "Please check number of passes " << endl;
  }

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

  for (const auto &patch : patches) {
    patchesViewId.push_back(patch.viewId);
  } // initialize patchesViewId

  ///////////////
  // Ordering views based on their distance & angle to target view
  ///////////////
  vector<size_t> sortedCamerasId(cameras.size());
  sortedCamerasId = sortViews(cameras, target);

  // Produce the individual pass synthesis results
  for (int passId = 0; passId < numberOfPasses; passId++) // Loop over NumberOfPasses
  {
    // Find the selected views for a given pass
    selectedViewsPass.clear();
    for (size_t id = 0; id < cameras.size(); ++id) {
      if (id < numberOfViewsPerPass[passId]) {
        selectedViewsPass.push_back(static_cast<unsigned int>(sortedCamerasId[id]));
      }
    }

    /////////////////
    // Update the Occupancy Map to be used in the Pass
    /////////////////
    for (size_t atlasId = 0; atlasId < maps.size(); ++atlasId) {
      transform(maps[atlasId].getPlane(0).begin(), maps[atlasId].getPlane(0).end(),
                mapsPass[passId][atlasId].getPlane(0).begin(), filterMaps);
    }

    ////////////////
    // Synthesis per pass
    ////////////////
    viewportPass[passId] =
        m_synthesizer->renderFrame(atlas, mapsPass[passId], patches, cameras, target);
  } // namespace TMIV::Renderer
  //////////////
  // Merging
  //////////////
  if (numberOfPasses > 1) {
    Texture444Depth16Frame mergedviewport = viewportPass[numberOfPasses - 1];
    for (auto passId = numberOfPasses - 1; passId > 0; passId--) {
      transform(viewportPass[passId - 1].second.getPlane(0).begin(),
                viewportPass[passId - 1].second.getPlane(0).end(),
                mergedviewport.second.getPlane(0).begin(),
                mergedviewport.second.getPlane(0).begin(), filterMergeDepth);

      for (auto i = 0; i < viewport.first.getNumberOfPlanes(); ++i) {
        my_transform(viewportPass[passId - 1].first.getPlane(i).begin(),
                     viewportPass[passId - 1].first.getPlane(i).end(),
                     mergedviewport.first.getPlane(i).begin(),
                     viewportPass[passId - 1].second.getPlane(0).begin(),
                     mergedviewport.second.getPlane(0).begin(),
                     mergedviewport.first.getPlane(i).begin(), filterMergeTexture);
      }
    }
    viewport = mergedviewport; // Final Merged

  } else {
    viewport = viewportPass[numberOfPasses - 1]; // Single Pass
  }

  m_inpainter->inplaceInpaint(viewport, target);
  return viewport;
}

Texture444Depth16Frame
MultipassRenderer::renderFrame(const MVD10Frame &atlas,
                               const Metadata::CameraParametersList &cameras,
                               const Metadata::CameraParameters &target) const {
  auto viewport = m_synthesizer->renderFrame(atlas, cameras, target);
  m_inpainter->inplaceInpaint(viewport, target);
  return viewport;
}
} // namespace TMIV::Renderer
