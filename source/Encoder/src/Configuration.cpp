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
using MivBitstream::PtlLevelIdc;
using MivBitstream::PtlProfileCodecGroupIdc;
using MivBitstream::PtlProfileReconstructionIdc;
using MivBitstream::PtlProfileToolsetIdc;

Configuration::Configuration(const Common::Json &rootNode, const Common::Json &componentNode)
    : intraPeriod{rootNode.require("intraPeriod").as<int32_t>()}
    , blockSizeDepthQualityDependent{rootNode.require("blockSizeDepthQualityDependent")
                                         .asVec<int32_t, 2>()}
    , haveTexture{rootNode.require("haveTextureVideo").as<bool>()}
    , haveGeometry{rootNode.require("haveGeometryVideo").as<bool>()}
    , haveOccupancy{rootNode.require("haveOccupancyVideo").as<bool>()}
    , framePacking{rootNode.require("framePacking").as<bool>()}
    , oneViewPerAtlasFlag{rootNode.require("oneViewPerAtlasFlag").as<bool>()}
    , geometryScaleEnabledFlag{haveGeometry && haveTexture &&
                               rootNode.require("geometryScaleEnabledFlag").as<bool>()}
    , dilationIter{componentNode.require("dilate").as<int32_t>()}
    , dynamicDepthRange{rootNode.require("dynamicDepthRange").as<bool>()}
    , attributeOffsetFlag{haveTexture && rootNode.require("attributeOffsetEnabledFlag").as<bool>()}
    , colorCorrectionEnabledFlag{haveTexture &&
                                 rootNode.require("colorCorrectionEnabledFlag").as<bool>()}
    , randomAccess{rootNode.require("randomAccess").as<bool>()}
    , patchRedundancyRemoval{rootNode.require("patchRedundancyRemoval").as<bool>()}
    , numGroups{rootNode.require("numGroups").as<uint8_t>()}
    , maxEntityId{rootNode.require("maxEntityId").as<uint16_t>()}
    , halveDepthRange{dynamicDepthRange && rootNode.require("halveDepthRange").as<bool>()}
    , viewportCameraParametersSei{rootNode.require("viewportCameraParametersSei").as<bool>()}
    , viewportPositionSei{rootNode.require("viewportPositionSei").as<bool>()} {
  queryMainParameters(rootNode, componentNode);
  queryProfileTierLevelParameters(rootNode);
  queryBitDepthParameters(rootNode);
  querySeiParameters(rootNode);
  verifyValid();
}

void Configuration::queryMainParameters(const Common::Json &rootNode,
                                        const Common::Json &componentNode) {
  if (const auto &node = componentNode.optional("overrideAtlasFrameSizes")) {
    std::cout
        << "WARNING: Overriding atlas frame sizes is meant for internal/preliminary experiments "
           "only.\n";
    for (const auto &subnode : node.as<Common::Json::Array>()) {
      overrideAtlasFrameSizes.push_back(subnode.asVec<int32_t, 2>());
    }
  } else if (!oneViewPerAtlasFlag) {
    maxLumaSampleRate = rootNode.require("maxLumaSampleRate").as<double>();
    maxLumaPictureSize = rootNode.require("maxLumaPictureSize").as<int32_t>();
    maxAtlases = rootNode.require("maxAtlases").as<int32_t>();
    maxAtlases = maxAtlases / std::max(1, int32_t{numGroups});
  }

  if (haveGeometry && !haveOccupancy) {
    depthOccThresholdIfSet = componentNode.require("depthOccThresholdIfSet").as<double>();

    if (!(0.0 < depthOccThresholdIfSet)) {
      throw std::runtime_error("The depthOccThresholdIfSet parameter is only used when the encoder "
                               "needs to use occupancy. The value 0 is not allowed.");
    }
    if (0.5 <= depthOccThresholdIfSet) {
      throw std::runtime_error(
          "The encoder takes a margin equal to the depth occupancy threshold, so "
          "setting the threshold this high will make it impossible to encode depth. Note that "
          "depthOccThresholdIfSet is normalized on the max. geometry sample value.");
    }
  }

  if (!haveGeometry) {
    dqParamsPresentFlag = rootNode.require("dqParamsPresentFlag").as<bool>();
  }

  if (attributeOffsetFlag) {
    attributeOffsetBitCount = rootNode.require("attributeOffsetBitCount").as<uint32_t>();
  }

  // Read the entity encoding range if exists
  if (0 < maxEntityId) {
    entityEncRange = rootNode.require("EntityEncodeRange").asVec<Common::SampleValue, 2>();
  }

  if (const auto &node = rootNode.optional("depthLowQualityFlag")) {
    depthLowQualityFlag = node.as<bool>();
  }
}

namespace {
template <typename Idc, size_t N>
auto queryIdc(const Common::Json &node, const std::string &key, const std::string &name,
              const std::array<Idc, N> &known) {
  const auto text = node.require(key).as<std::string>();

  for (auto i : known) {
    if (fmt::format("{}", i) == text) {
      return i;
    }
  }
  throw std::runtime_error(fmt::format("The configured {} IDC {} is unknown", name, text));
}
} // namespace

void Configuration::queryProfileTierLevelParameters(const Common::Json &rootNode) {
  codecGroupIdc =
      queryIdc(rootNode, "codecGroupIdc", "codec group", MivBitstream::knownCodecGroupIdcs);
  toolsetIdc = queryIdc(rootNode, "toolsetIdc", "toolset", MivBitstream::knownToolsetIdcs);
  reconstructionIdc = queryIdc(rootNode, "reconstructionIdc", "reconstruction",
                               MivBitstream::knownReconstructionIdcs);
  levelIdc = queryIdc(rootNode, "levelIdc", "level", MivBitstream::knownLevelIdcs);
}

void Configuration::queryBitDepthParameters(const Common::Json &rootNode) {
  if (haveOccupancy) {
    occBitDepth = rootNode.require("bitDepthOccupancyVideo").as<uint32_t>();
  }

  if (haveGeometry) {
    geoBitDepth = rootNode.require("bitDepthGeometryVideo").as<uint32_t>();
  }

  if (haveTexture) {
    texBitDepth = rootNode.require("bitDepthTextureVideo").as<uint32_t>();
  }

  if (framePacking) {
    pacBitDepth = std::max({occBitDepth, geoBitDepth, texBitDepth});
  }
}

void Configuration::querySeiParameters(const Common::Json &rootNode) {
  if (const auto &node = rootNode.optional("ViewingSpace")) {
    viewingSpace = MivBitstream::ViewingSpace::loadFromJson(node, rootNode);
  }
}

void Configuration::verifyValid() const {
  for (const auto blockSize : blockSizeDepthQualityDependent) {
    VERIFY(2 <= blockSize);
    VERIFY((blockSize & (blockSize - 1)) == 0);
  }

  VERIFY(intraPeriod <= maxIntraPeriod);
}
} // namespace TMIV::Encoder
