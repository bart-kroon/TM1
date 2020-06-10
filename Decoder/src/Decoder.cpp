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

#include <TMIV/Decoder/Decoder.h>

#include <TMIV/Common/Factory.h>
#include <TMIV/Decoder/GeometryScaler.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
//#include <fstream>
//#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;
using namespace TMIV::Renderer;

namespace TMIV::Decoder {
Decoder::Decoder(const Json &rootNode, const Json &componentNode)
    : m_geometryScaler{rootNode, componentNode}
    , m_entityBasedPatchMapFilter{rootNode, componentNode} {
  m_culler = Factory<ICuller>::getInstance().create("Culler", rootNode, componentNode);
  m_renderer = Factory<IRenderer>::getInstance().create("Renderer", rootNode, componentNode);
}

namespace {
void checkRestrictions(const AccessUnit &frame) {
  if (frame.vps->vps_miv_extension_flag()) {
    const auto &vme = frame.vps->vps_miv_extension();
    if (vme.vme_vui_params_present_flag()) {
      const auto &mvp = vme.miv_vui_parameters();
      if (!mvp.coordinate_axis_system_params().isOmafCas()) {
        throw runtime_error(
            "The VUI indicates that a coordinate axis system other than that of OMAF is used. The "
            "TMIV decoder/renderer is not yet able to convert between coordinate axis systems.");
      }
    }
  }
}
/*
void writeOccupancy(const std::string &path, const Common::Mask &frame, int frameIndex) {

  std::cout << "Writing occupancyAtlasMap " << frameIndex << " to " << path << '\n';

  std::fstream stream(path, frameIndex == 0 ? std::ios::out | std::ios::binary
                                            : std::ios::in | std::ios::out | std::ios::binary);
  if (!stream.good()) {
    throw std::runtime_error("Failed to open file for writing");
  }

  stream.seekp(int64_t(frameIndex) * frame.getDiskSize());

  frame.dump(stream);
  int bytes = frame.getDiskSize() - frame.getMemorySize();
  constexpr auto fillValue = Common::neutralColor<YUV420P8>();
  const auto padding = std::vector(bytes / sizeof(fillValue), fillValue);
  auto buffer = std::vector<char>(bytes);
  std::memcpy(buffer.data(), padding.data(), buffer.size());
  stream.write(buffer.data(), buffer.size());

  if (!stream.good()) {
    throw std::runtime_error("Failed to write to file");
  }
}

void writeGeometry(const std::string &path, const Common::Depth10Frame &frame, int frameIndex) {

  std::cout << "Writing geometry of " << frameIndex << " to " << path << '\n';

  std::fstream stream(path, frameIndex == 0 ? std::ios::out | std::ios::binary
                                            : std::ios::in | std::ios::out | std::ios::binary);
  if (!stream.good()) {
    throw std::runtime_error("Failed to open file for writing");
  }

  stream.seekp(int64_t(frameIndex) * frame.getDiskSize());

  frame.dump(stream);
  int bytes = frame.getDiskSize() - frame.getMemorySize();
  constexpr auto fillValue = Common::neutralColor<YUV420P10>();
  const auto padding = std::vector(bytes / sizeof(fillValue), fillValue);
  auto buffer = std::vector<char>(bytes);
  std::memcpy(buffer.data(), padding.data(), buffer.size());
  stream.write(buffer.data(), buffer.size());

  if (!stream.good()) {
    throw std::runtime_error("Failed to write to file");
  }
}
*/
void extractOccupancy(AccessUnit &frame) {
  bool explicitOccupancyMode = false;
  for (auto i = 0; i <= frame.vps->vps_atlas_count_minus1(); i++)
    if (frame.vps->vps_occupancy_video_present_flag(i))
      explicitOccupancyMode = true;

  for (auto i = 0; i <= frame.vps->vps_atlas_count_minus1(); i++) {
    auto &atlas = frame.atlas[i];
    atlas.occFrame = Mask{atlas.frameSize().x(), atlas.frameSize().y()};
    if (!frame.vps->vps_occupancy_video_present_flag(i)) {
      atlas.occFrame.fillOne();
      if (!explicitOccupancyMode) {
        // embedded occupancy case: Implementation assumes geoFrame (full size depth) is available
        for (auto y = 0; y < atlas.frameSize().y(); y++)
          for (auto x = 0; x < atlas.frameSize().x(); x++) {
            auto patchId = atlas.patchId(y, x);
            if (patchId == unusedPatchId)
              continue;
            const auto &patchParams = atlas.patchParamsList[patchId];
            const auto &viewParams =
                atlas.viewParamsList[atlas.patchParamsList[patchId].pduViewId()];
            const auto occupancyTransform = OccupancyTransform{viewParams, patchParams};

            if (!occupancyTransform.occupant(atlas.geoFrame.getPlane(0)(y, x)))
              frame.atlas[i].occFrame.getPlane(0)(y, x) = 0;
          }
      }
    } else {
      // Account for padding in case done to the coded occupancy video
      Vec2i decOccSize = atlas.decOccFrameSize(*frame.vps);
      int occScaleX = ceil((float)atlas.occFrame.getWidth() / (float)decOccSize[0]);
      int origWidth = atlas.occFrame.getWidth() / occScaleX;
      int occScaleY = ceil((float)atlas.occFrame.getHeight() / (float)decOccSize[1]);
      int origHeight = atlas.occFrame.getHeight() / occScaleY;
      int xPad = origWidth % 2;
      int yPad = origHeight % 2;
      // upscale Nearest Neighbor (External occupancy coding case)
      for (int yo = 0; yo < atlas.occFrame.getHeight(); ++yo) {
        for (int xo = 0; xo < atlas.occFrame.getWidth(); ++xo) {
          const auto xi = xo * (decOccSize[0] - xPad) / atlas.occFrame.getWidth();
          const auto yi = yo * (decOccSize[1] - yPad) / atlas.occFrame.getHeight();
          atlas.occFrame.getPlane(0)(yo, xo) = atlas.decOccFrame.getPlane(0)(yi, xi);
        }
      }
    }
    //writeOccupancy("F:/Test_TO_c" + to_string(i) + (string) "_1920x4640_yuv420p_ExpOcc.yuv", frame.atlas[i].occFrame, 0);
    //writeGeometry("F:/Test_TG_c" + to_string(i) + (string) "_1920x4640_yuv420p10le_ExpOcc.yuv", frame.atlas[i].geoFrame, 0);
  }
}
} // namespace

auto Decoder::decodeFrame(AccessUnit &frame, const ViewParams &viewportParams) const
    -> Texture444Depth16Frame {
  checkRestrictions(frame);
  m_geometryScaler.inplaceScale(frame);
  extractOccupancy(frame);
  m_entityBasedPatchMapFilter.inplaceFilterBlockToPatchMaps(frame);
  m_culler->inplaceFilterBlockToPatchMaps(frame, viewportParams);
  return m_renderer->renderFrame(frame, viewportParams);
}
} // namespace TMIV::Decoder
