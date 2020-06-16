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

#include <TMIV/GeometryQuantizer/ExplicitOccupancy.h>

#include <TMIV/MivBitstream/DepthOccupancyTransform.h>

#include <iostream>
#include <stdexcept>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::GeometryQuantizer {
ExplicitOccupancy::ExplicitOccupancy(const Json & /*unused*/, const Json &componentNode) {
  if (auto subnode = componentNode.optional("isAtlasCompletePerGroupFlag")) {
    for (size_t i = 0; i < subnode.size(); ++i) {
      m_isAtlasCompleteFlag.push_back(subnode.at(i).asBool());
    }
  }
}

auto ExplicitOccupancy::transformSequenceParams(MivBitstream::IvSequenceParams sequenceParams)
    -> const MivBitstream::IvSequenceParams & {
  m_inSequenceParams = move(sequenceParams);
  m_outSequenceParams = m_inSequenceParams;

  for (uint8_t i = 0; i <= m_outSequenceParams.vps.vps_atlas_count_minus1(); i++)
    if (m_isAtlasCompleteFlag.size() > i)
      m_outSequenceParams.vps.vps_occupancy_video_present_flag(i, !m_isAtlasCompleteFlag[i]);
    else
      m_outSequenceParams.vps.vps_occupancy_video_present_flag(i, true);
  m_depthLowQualityFlag=m_outSequenceParams.vme().vme_depth_low_quality_flag();
  return m_outSequenceParams;
}

auto ExplicitOccupancy::transformAccessUnitParams(MivBitstream::IvAccessUnitParams accessUnitParams)
    -> const MivBitstream::IvAccessUnitParams & {
  m_accessUnitParams = accessUnitParams;
  return m_accessUnitParams;
}

auto ExplicitOccupancy::transformAtlases(const Common::MVD16Frame &inAtlases)
    -> Common::MVD10Frame {
  auto outAtlases = MVD10Frame{};
  outAtlases.reserve(inAtlases.size());

  for (const auto &inAtlas : inAtlases) {
    outAtlases.emplace_back(inAtlas.texture,
                            Depth10Frame{inAtlas.depth.getWidth(), inAtlas.depth.getHeight()},
                            inAtlas.occupancy);
  }

  for (const auto &patch : m_accessUnitParams.patchParamsList) {
    const auto &inViewParams = m_inSequenceParams.viewParamsList[patch.pduViewId()];
    const auto &outViewParams = m_outSequenceParams.viewParamsList[patch.pduViewId()];
    const auto inOccupancyTransform = OccupancyTransform{inViewParams};
#ifndef NDEBUG
    const auto outOccupancyTransform = OccupancyTransform{outViewParams, patch};
#endif
    const auto inDepthTransform = DepthTransform<16>{inViewParams.dq};
    const auto outDepthTransform = DepthTransform<10>{outViewParams.dq, patch};

    for (auto i = 0; i < patch.pdu2dSize().y(); ++i) {
      for (auto j = 0; j < patch.pdu2dSize().x(); ++j) {
        const auto n = i + patch.pdu2dPos().y();
        const auto m = j + patch.pdu2dPos().x();

        const auto &plane = inAtlases[patch.vuhAtlasId].depth.getPlane(0);

        if (n < 0 || n >= int(plane.height()) || m < 0 || m >= int(plane.width())) {
          abort();
        }

        const auto inLevel = plane(n, m);

        if (inOccupancyTransform.occupant(inLevel)) {
          const auto normDisp = inDepthTransform.expandNormDisp(inLevel);
          const auto outLevel = outDepthTransform.quantizeNormDisp(normDisp, 0);
          assert(outOccupancyTransform.occupant(outLevel));

          outAtlases[patch.vuhAtlasId].depth.getPlane(0)(n, m) = outLevel;
        }
      }
    }
  }

  // Apply depth padding
   if (!m_depthLowQualityFlag)
	   padGeometryFromLeft(outAtlases);
  // padGeometryWithAvg(outAtlases);
  // padGeometryWithBlockAvg(outAtlases);
  // padGeometryFromTopLeft(outAtlases);
  // padGeometryFromLeft(outAtlases);
  // padGeometryWithMidRange(outAtlases);

  return outAtlases;
}

void ExplicitOccupancy::padGeometryFromLeft(MVD10Frame &atlases) {
  for (uint8_t i = 0; i < atlases.size(); ++i) {
    if (m_outSequenceParams.vps.vps_occupancy_video_present_flag(uint8_t(i))) {
      auto &depthAtlasMap = atlases[i].depth;
      int depthScale[2] = {
          m_accessUnitParams.atlas[i].asps.asps_frame_height() / depthAtlasMap.getHeight(),
          m_accessUnitParams.atlas[i].asps.asps_frame_width() / depthAtlasMap.getWidth()};
      const auto &occupancyAtlasMap = atlases[i].occupancy;
      int yOcc, xOcc;
      for (int y = 0; y < depthAtlasMap.getHeight(); y++) {
        for (int x = 1; x < depthAtlasMap.getWidth(); x++) {
          auto depth = depthAtlasMap.getPlane(0)(y, x);
          yOcc = y >> m_accessUnitParams.atlas[i].asps.asps_log2_patch_packing_block_size();
          xOcc = x >> m_accessUnitParams.atlas[i].asps.asps_log2_patch_packing_block_size();
          if (occupancyAtlasMap.getPlane(0)(yOcc, xOcc) == 0 ||
              (depth == 0 &&
               atlases[i].texture.getPlane(0)(y * depthScale[1], x * depthScale[0]) == 512)) {
            depthAtlasMap.getPlane(0)(y, x) = depthAtlasMap.getPlane(0)(y, x - 1);
          }
        }
      }
    }
  }
}
/*
void ExplicitOccupancy::padGeometryWithBlockAvg(MVD10Frame &atlases) {
  int codingUnitSize[] = {64, 64}; // MaxCUWidth & MaxCUHeight per HM config
  for (uint8_t i = 0; i < atlases.size(); ++i) {
    if (m_outSequenceParams.vps.vps_occupancy_video_present_flag(uint8_t(i))) {
      auto &depthAtlasMap = atlases[i].depth;
      Mask padMask = Mask{depthAtlasMap.getWidth(), depthAtlasMap.getHeight()};
      const auto &occupancyAtlasMap = atlases[i].occupancy;
      int yOcc, xOcc;
      for (int y = 0; y < depthAtlasMap.getHeight(); y++) {
        for (int x = 0; x < depthAtlasMap.getWidth(); x++) {
          yOcc = y >> m_accessUnitParams.atlas[i].asps.asps_log2_patch_packing_block_size();
          xOcc = x >> m_accessUnitParams.atlas[i].asps.asps_log2_patch_packing_block_size();
          if (occupancyAtlasMap.getPlane(0)(yOcc, xOcc)) {
            padMask.getPlane(0)(y, x) = 1;
          }
        }
      }
      for (int y = 0; y < depthAtlasMap.getHeight(); y = y + codingUnitSize[1]) {
        for (int x = 0; x < depthAtlasMap.getWidth(); x = x + codingUnitSize[0]) {
          double blcokAvg = 0;
          int count = 0;
          int blockEnd[] = {min(x + codingUnitSize[0], depthAtlasMap.getWidth()),
                            min(y + codingUnitSize[1], depthAtlasMap.getHeight())};
          for (int yBlock = y; yBlock < blockEnd[1]; yBlock++) {
            for (int xBlock = x; xBlock < blockEnd[0]; xBlock++) {
              blcokAvg = blcokAvg + (double)padMask.getPlane(0)(yBlock, xBlock) *
                                        depthAtlasMap.getPlane(0)(yBlock, xBlock);
              count = count + padMask.getPlane(0)(yBlock, xBlock);
            }
          }
          blcokAvg = blcokAvg / count;
          if (count > 0) {
            for (int yBlock = y; yBlock < blockEnd[1]; yBlock++) {
              for (int xBlock = x; xBlock < blockEnd[0]; xBlock++) {
                if (!padMask.getPlane(0)(yBlock, xBlock))
                  depthAtlasMap.getPlane(0)(yBlock, xBlock) = (uint16_t)blcokAvg;
              }
            }
          }
        }
      }
    }
  }
}

void ExplicitOccupancy::padGeometryFromLeft(MVD10Frame &atlases) {
for (uint8_t i = 0; i < atlases.size(); ++i) {
  if (m_outSequenceParams.vps.vps_occupancy_video_present_flag(uint8_t(i))) {
    auto &depthAtlasMap = atlases[i].depth;
    int depthScale[2] = {
        m_accessUnitParams.atlas[i].asps.asps_frame_height() / depthAtlasMap.getHeight(),
        m_accessUnitParams.atlas[i].asps.asps_frame_width() / depthAtlasMap.getWidth()};
    const auto &occupancyAtlasMap = atlases[i].occupancy;
    int yOcc, xOcc;
    for (int y = 0; y < depthAtlasMap.getHeight(); y++) {
      for (int x = 1; x < depthAtlasMap.getWidth(); x++) {
        auto depth = depthAtlasMap.getPlane(0)(y, x);
        yOcc = y >> m_accessUnitParams.atlas[i].asps.asps_log2_patch_packing_block_size();
        xOcc = x >> m_accessUnitParams.atlas[i].asps.asps_log2_patch_packing_block_size();
        if (occupancyAtlasMap.getPlane(0)(yOcc, xOcc) == 0 ||
            (depth == 0 &&
             atlases[i].texture.getPlane(0)(y * depthScale[1], x * depthScale[0]) == 512)) {
          depthAtlasMap.getPlane(0)(y, x) = depthAtlasMap.getPlane(0)(y, x - 1);
        }
      }
    }
  }
}
}

void ExplicitOccupancy::padGeometryWithMidRange(MVD10Frame &atlases) {
  for (uint8_t i = 0; i < atlases.size(); ++i) {
    if (m_outSequenceParams.vps.vps_occupancy_video_present_flag(uint8_t(i))) {
      auto &depthAtlasMap = atlases[i].depth;
      int depthScale[2] = {
          m_accessUnitParams.atlas[i].asps.asps_frame_height() / depthAtlasMap.getHeight(),
          m_accessUnitParams.atlas[i].asps.asps_frame_width() / depthAtlasMap.getWidth()};
      const auto &occupancyAtlasMap = atlases[i].occupancy;
      int yOcc, xOcc;
      for (int y = 0; y < depthAtlasMap.getHeight(); y++) {
        for (int x = 0; x < depthAtlasMap.getWidth(); x++) {
          auto depth = depthAtlasMap.getPlane(0)(y, x);
          yOcc = y >> m_accessUnitParams.atlas[i].asps.asps_log2_patch_packing_block_size();
          xOcc = x >> m_accessUnitParams.atlas[i].asps.asps_log2_patch_packing_block_size();
          if (occupancyAtlasMap.getPlane(0)(yOcc, xOcc) == 0 ||
              (depth == 0 &&
               atlases[i].texture.getPlane(0)(y * depthScale[1], x * depthScale[0]) == 512)) {
            depthAtlasMap.getPlane(0)(y, x) = 512;
          }
        }
      }
    }
  }
}

void ExplicitOccupancy::padGeometryFromTopLeft(MVD10Frame &atlases) {
  for (uint8_t i = 0; i < atlases.size(); ++i) {
    if (m_outSequenceParams.vps.vps_occupancy_video_present_flag(uint8_t(i))) {
      auto &depthAtlasMap = atlases[i].depth;
      int depthScale[2] = {
          m_accessUnitParams.atlas[i].asps.asps_frame_height() / depthAtlasMap.getHeight(),
          m_accessUnitParams.atlas[i].asps.asps_frame_width() / depthAtlasMap.getWidth()};
      const auto &occupancyAtlasMap = atlases[i].occupancy;
      int yOcc, xOcc;
      uint32_t avgValue;
      for (int y = 1; y < depthAtlasMap.getHeight(); y++) {
        for (int x = 1; x < depthAtlasMap.getWidth(); x++) {
          auto depth = depthAtlasMap.getPlane(0)(y, x);
          yOcc = y >> m_accessUnitParams.atlas[i].asps.asps_log2_patch_packing_block_size();
          xOcc = x >> m_accessUnitParams.atlas[i].asps.asps_log2_patch_packing_block_size();
          if (occupancyAtlasMap.getPlane(0)(yOcc, xOcc) == 0 ||
              (depth == 0 &&
               atlases[i].texture.getPlane(0)(y * depthScale[1], x * depthScale[0]) == 512)) {
            avgValue = ((uint32_t)depthAtlasMap.getPlane(0)(y, x - 1) +
                       (uint32_t)depthAtlasMap.getPlane(0)(y - 1, x) +
                       (uint32_t)depthAtlasMap.getPlane(0)(y - 1, x - 1))/3;
            depthAtlasMap.getPlane(0)(y, x) = (uint16_t) avgValue;
          }
        }
      }
    }
  }
}

static vector<uint16_t> avg;
static bool isFirstFrame = true;
void ExplicitOccupancy::padGeometryWithAvg(MVD10Frame &atlases) {
  if (isFirstFrame)
    for (uint8_t i = 0; i < atlases.size(); ++i)
      avg.push_back(0);

  for (uint8_t i = 0; i < atlases.size(); ++i) {
    if (m_outSequenceParams.vps.vps_occupancy_video_present_flag(uint8_t(i))) {
      auto &depthAtlasMap = atlases[i].depth;
      int depthScale[2] = {
          m_accessUnitParams.atlas[i].asps.asps_frame_height() / depthAtlasMap.getHeight(),
          m_accessUnitParams.atlas[i].asps.asps_frame_width() / depthAtlasMap.getWidth()};
      const auto &occupancyAtlasMap = atlases[i].occupancy;
      int yOcc, xOcc;
      if (isFirstFrame) {
        // Find Average geometry value per atlas at first frame in IRAP only
        double sum = 0, count = 0;
        for (int y = 0; y < depthAtlasMap.getHeight(); y++) {
          for (int x = 0; x < depthAtlasMap.getWidth(); x++) {
            auto depth = depthAtlasMap.getPlane(0)(y, x);
            yOcc = y >> m_accessUnitParams.atlas[i].asps.asps_log2_patch_packing_block_size();
            xOcc = x >> m_accessUnitParams.atlas[i].asps.asps_log2_patch_packing_block_size();
            if (occupancyAtlasMap.getPlane(0)(yOcc, xOcc) > 0 && depth != 0) {
              sum = sum + (double)depth;
              count++;
            }
          }
        }
        avg[i] = sum / count;
        cout << "Padded depth value is " << avg[i] << endl;
      }
      for (int y = 0; y < depthAtlasMap.getHeight(); y++) {
        for (int x = 0; x < depthAtlasMap.getWidth(); x++) {
          auto depth = depthAtlasMap.getPlane(0)(y, x);
          yOcc = y >> m_accessUnitParams.atlas[i].asps.asps_log2_patch_packing_block_size();
          xOcc = x >> m_accessUnitParams.atlas[i].asps.asps_log2_patch_packing_block_size();
          if (occupancyAtlasMap.getPlane(0)(yOcc, xOcc) == 0 ||
              (depth == 0 &&
               atlases[i].texture.getPlane(0)(y * depthScale[1], x * depthScale[0]) == 512)) {
            depthAtlasMap.getPlane(0)(y, x) = avg[i];
          }
        }
      }
    }
  }
  isFirstFrame = false;
}
*/
} // namespace TMIV::GeometryQuantizer
