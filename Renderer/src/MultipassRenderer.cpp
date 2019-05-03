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
    m_NumberOfPasses = subnode.asInt();
  if (auto subnode = componentNode.optional("NumberOfViewsPerPass")) {
    if (subnode) {
      for (auto i = 0u; i != subnode.size(); i++)
        m_NumberOfViewsPerPass.push_back(subnode.at(i).asInt());
    }
  }
}

Common::Texture444Depth10Frame
MultipassRenderer::renderFrame(const Common::MVD10Frame &atlas,
                               const Common::PatchIdMapList &maps,
                               const Metadata::AtlasParametersList &patches,
                               const Metadata::CameraParametersList &cameras,
                               const Metadata::CameraParameters &target) const {

	// Intel Hybrid
  const int NumberOfPasses =
      TMIV::Renderer::MultipassRenderer::m_NumberOfPasses;

  Common::Texture444Depth10Frame viewport;
  Common::Texture444Depth10Frame viewportPass[3];
  // MVD10Frame viewportPass;
  Common::PatchIdMapList mapsPass[3];

  for (auto j = 0; j < NumberOfPasses; j++) {
    for (auto k = 0; k < atlas.size(); k++) {
      PatchIdMap patchMap(maps[k].getWidth(), maps[k].getHeight());
      std::fill(patchMap.getPlane(0).begin(), patchMap.getPlane(0).end(),
                unusedPatchId);
      mapsPass[j].push_back(patchMap);
    }
  } // initalize mapsPass by 0xFFFF

  // Ordering views based on their distance & angle to target view
  float x_target = target.position[0];
  float y_target = target.position[1];
  float z_target = target.position[2];
  float yaw_target = target.rotation[0];
  float pitch_target = target.rotation[1];
  float roll_target = target.rotation[2];
  constexpr float radperdeg{0.01745329251994329576923690768489f};
  vector<float> Distance, Angle, AngleWeight;
  for (auto id = 0u; id < cameras.size(); id++) {
    Distance.push_back(sqrt(pow(cameras[id].position[0] - x_target, 2) +
                            pow(cameras[id].position[1] - y_target, 2) +
                            pow(cameras[id].position[2] - z_target, 2)));
    Angle.push_back(1 / radperdeg *
                    acos(sin(cameras[id].rotation[1] * radperdeg) *
                             sin(pitch_target * radperdeg) +
                         cos(cameras[id].rotation[1] * radperdeg) *
                             cos(pitch_target * radperdeg) *
                             cos((cameras[id].rotation[0] - yaw_target) *
                                 radperdeg))); // Angle is in degree unit
    if (Angle[id] > 180)
      Angle[id] =
          Angle[id] - 360; // to assure angle is ranging from -180 to 180 degree
    // Introduce AngleWeight as a simple triangle function (with value of 1 when
    // angle is 0 & value of 0 when angle is 180)
    if (Angle[id] > 0.0)
      AngleWeight.push_back(-1 / 180 * Angle[id] + 1);
    else
      AngleWeight.push_back(1 / 180 * Angle[id] + 1);
  }
  // Find the sorted cameras indices
  vector<size_t> SortedCamerasId(cameras.size());
  iota(SortedCamerasId.begin(), SortedCamerasId.end(), 0); // initalization
  sort(SortedCamerasId.begin(), SortedCamerasId.end(),
       [&Distance, &AngleWeight](size_t i1, size_t i2) {
         return Distance[i1] * AngleWeight[i1] < Distance[i2] * AngleWeight[i2];
       });

  for (auto passId = 0; passId < NumberOfPasses;
       passId++) // Loop over NumberOfPasses
  {
    // Find the selected views for a given pass
    vector<unsigned int> SelectedViewsPass;
    for (auto id = 0u; id < cameras.size(); id++) {
      if (SortedCamerasId[id] <
          TMIV::Renderer::MultipassRenderer::m_NumberOfViewsPerPass[passId])
        SelectedViewsPass.push_back(id);
    }

    // Update the Occupancy Map to be used in the Pass
    /*
    Common::PatchIdMap ids = maps[0];

        std::copy_if(ids.getPlane(0).begin(), ids.getPlane(0).end(),
                 std::back_inserter(v),
                 [len](std::uint16_t i) { return len == i.length(); });

        mapsPass[0][0] = [&ids](int index) {

        }
        */
    /*
   maps;
   const Mat<uint16_t> &ids
   auto addTriangle = [&result, &ids](int v0, int v1, int v2) {
     const int id0 = ids[v0];
     if (id0 == unusedPatchId || id0 != ids[v1] || id0 != ids[v2]) {
       return;
     }
     result.push_back({{v0, v1, v2}, 0.5f});
   };
           */

    for (auto patchId = 0; patchId < patches.size(); patchId++) {
      // Access OccupancyMap and copy only Id of patches that belong to the
      // selected views @ passId
      for (auto SelectedViewIndex = 0;
           SelectedViewIndex < SelectedViewsPass.size(); SelectedViewIndex++) {

        if (patches[patchId].viewId == SelectedViewsPass[SelectedViewIndex]) {
          auto atlasId = patches[patchId].atlasId;
          auto patch = patches[patchId];

          const Vec2i &q0 = patch.posInAtlas;
          int w = patch.patchSize.x(), h = patch.patchSize.y();
          bool isRotated = (patch.rotation != Metadata::PatchRotation::upright);
          int xMin = q0.x(), xLast = q0.x() + (isRotated ? h : w);
          int yMin = q0.y(), yLast = q0.y() + (isRotated ? w : h);
          for (auto y = yMin; y < yLast; y++)
            std::fill(mapsPass[passId][atlasId].getPlane(0).row_begin(y) + xMin,
                      mapsPass[passId][atlasId].getPlane(0).row_begin(y) +
                          xLast,
                      patchId);
        }
      }
    }
    viewportPass[passId] = m_synthesizer->renderFrame(atlas, mapsPass[passId], patches, cameras, target); // cameras or camerasPass ??
  }                                         // namespace TMIV::Renderer

  viewport =
      viewportPass[NumberOfPasses - 1]; // Right now we are passing the final
                                        // pass results / mimicing single pass

  //auto viewport =
  //    m_synthesizer->renderFrame(atlas, maps, patches, cameras, target);
  // m_inpainter->inplaceInpaint(viewport, target);
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
