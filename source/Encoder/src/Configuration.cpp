/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#include <TMIV/Encoder/Configuration.h>
#include <TMIV/MivBitstream/SequenceConfig.h>

#include <TMIV/Common/verify.h>

namespace TMIV::Encoder {
Configuration::Configuration(const Common::Json &rootNode, const Common::Json &componentNode)
    : intraPeriod{rootNode.require("intraPeriod").as<int>()}
    , blockSizeDepthQualityDependent{rootNode.require("blockSizeDepthQualityDependent")
                                         .asVec<int, 2>()}
    , haveTexture{rootNode.require("haveTextureVideo").as<bool>()}
    , haveGeometry{rootNode.require("haveGeometryVideo").as<bool>()}
    , haveOccupancy{rootNode.require("haveOccupancyVideo").as<bool>()}
    , framePacking{rootNode.require("framePacking").as<bool>()}
    , oneViewPerAtlasFlag{rootNode.require("oneViewPerAtlasFlag").as<bool>()}
    , geometryScaleEnabledFlag{haveGeometry && haveTexture &&
                               rootNode.require("geometryScaleEnabledFlag").as<bool>()}
    , dilationIter{componentNode.require("dilate").as<int>()}
    , dynamicDepthRange{rootNode.require("dynamicDepthRange").as<bool>()}
    , attributeOffsetFlag{haveTexture && rootNode.require("attributeOffsetEnabledFlag").as<bool>()}
    , viewportCameraParametersSei{rootNode.require("viewportCameraParametersSei").as<bool>()}
    , viewportPositionSei{rootNode.require("viewportPositionSei").as<bool>()}
    , colorCorrectionEnabledFlag{haveTexture &&
                                 rootNode.require("colorCorrectionEnabledFlag").as<bool>()}
    , randomAccess{rootNode.require("randomAccess").as<bool>()}
    , patchRedundancyRemoval{rootNode.require("patchRedundancyRemoval").as<bool>()}
    , numGroups{rootNode.require("numGroups").as<uint8_t>()}
    , maxEntityId{rootNode.require("maxEntityId").as<uint16_t>()} {
  for (const auto blockSize : blockSizeDepthQualityDependent) {
    VERIFY(2 <= blockSize);
    VERIFY((blockSize & (blockSize - 1)) == 0);
  }

  if (const auto &node = componentNode.optional("overrideAtlasFrameSizes")) {
    std::cout
        << "WARNING: Overriding atlas frame sizes is meant for internal/preliminary experiments "
           "only.\n";
    for (const auto &subnode : node.as<Common::Json::Array>()) {
      overrideAtlasFrameSizes.push_back(subnode.asVec<int, 2>());
    }
  } else if (!oneViewPerAtlasFlag) {
    maxLumaSampleRate = rootNode.require("maxLumaSampleRate").as<double>();
    maxLumaPictureSize = rootNode.require("maxLumaPictureSize").as<int32_t>();
    maxAtlases = rootNode.require("maxAtlases").as<int>();
    maxAtlases = maxAtlases / std::max(1, int{numGroups});
  }

  if (haveGeometry && !haveOccupancy) {
    depthOccThresholdIfSet =
        componentNode.require("depthOccThresholdIfSet").as<Common::SampleValue>();

    if (depthOccThresholdIfSet < 1) {
      throw std::runtime_error("The depthOccThresholdIfSet parameter is only used when the encoder "
                               "needs to use occupancy. The value 0 is not allowed.");
    }
    if (depthOccThresholdIfSet >= 500) {
      throw std::runtime_error(
          "The encoder takes a margin equal to the threshold, so "
          "setting the threshold this high will make it impossible to encode depth.");
    }
  }

  if (!haveGeometry) {
    dqParamsPresentFlag = rootNode.require("dqParamsPresentFlag").as<bool>();
  }

  if (attributeOffsetFlag) {
    attributeOffsetBitCount = rootNode.require("attributeOffsetBitCount").as<int>();
  }

  // Read the entity encoding range if exists
  if (0 < maxEntityId) {
    entityEncRange = rootNode.require("EntityEncodeRange").asVec<Common::SampleValue, 2>();
  }

  if (const auto &node = rootNode.optional("depthLowQualityFlag")) {
    depthLowQualityFlag = node.as<bool>();
  }

  if (const auto &node = rootNode.optional("ViewingSpace")) {
    viewingSpace = MivBitstream::ViewingSpace::loadFromJson(node, rootNode);
  }

  VERIFY(intraPeriod <= maxIntraPeriod);

  using CodecGroupIdc = MivBitstream::PtlProfileCodecGroupIdc;
  codecGroupIdc = [&rootNode]() {
    const auto text = rootNode.require("codecGroupIdc").as<std::string>();
    for (auto i : {CodecGroupIdc::AVC_Progressive_High, CodecGroupIdc::HEVC_Main10,
                   CodecGroupIdc::HEVC444, CodecGroupIdc::VVC_Main10, CodecGroupIdc::MP4RA}) {
      if (fmt::format("{}", i) == text) {
        return i;
      }
    }
    throw std::runtime_error(fmt::format("The configured codec group IDC {} is unknown", text));
  }();

  using ToolsetIdc = MivBitstream::PtlProfilePccToolsetIdc;
  toolsetIdc = [&rootNode]() {
    const auto text = rootNode.require("toolsetIdc").as<std::string>();
    for (auto i : {ToolsetIdc::VPCC_Basic, ToolsetIdc::VPCC_Extended, ToolsetIdc::MIV_Main,
                   ToolsetIdc::MIV_Extended, ToolsetIdc::MIV_Geometry_Absent}) {
      if (fmt::format("{}", i) == text) {
        return i;
      }
    }
    throw std::runtime_error(fmt::format("The configured toolset IDC {} is unknown", text));
  }();

  switch (toolsetIdc) {
  case ToolsetIdc::MIV_Main:
    VERIFY(haveGeometry && !haveOccupancy);
    break;
  case ToolsetIdc::MIV_Extended:
    break;
  case ToolsetIdc::MIV_Geometry_Absent:
    VERIFY(!haveGeometry && !haveOccupancy);
    break;
  default:
    throw std::runtime_error(fmt::format("The {} toolset IDC is not supported", toolsetIdc));
  }
}
} // namespace TMIV::Encoder
