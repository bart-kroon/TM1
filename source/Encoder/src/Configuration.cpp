/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#include "Configuration.h"

#include <TMIV/Common/LoggingStrategy.h>
#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/Formatters.h>
#include <TMIV/MivBitstream/SequenceConfig.h>

namespace TMIV::Encoder {
using MivBitstream::PtlLevelIdc;
using MivBitstream::PtlProfileCodecGroupIdc;
using MivBitstream::PtlProfileReconstructionIdc;
using MivBitstream::PtlProfileToolsetIdc;

Configuration::Configuration(const Common::Json &componentNode)
    : intraPeriod{componentNode.require("intraPeriod").as<int32_t>()}
    , interPeriod{intraPeriod}
    , blockSize{componentNode.require("blockSize").as<int32_t>()}
    , haveTexture{componentNode.require("haveTextureVideo").as<bool>()}
    , haveGeometry{componentNode.require("haveGeometryVideo").as<bool>()}
    , haveOccupancy{componentNode.require("haveOccupancyVideo").as<bool>()}
    , oneViewPerAtlasFlag{componentNode.require("oneViewPerAtlasFlag").as<bool>()}
    , geometryScaleEnabledFlag{haveGeometry && haveTexture &&
                               componentNode.require("geometryScaleEnabledFlag").as<bool>()}
    , chromaScaleEnabledFlag{haveTexture &&
                             componentNode.require("chromaScaleEnabledFlag").as<bool>()}
    , textureOffsetFlag{haveTexture && componentNode.require("textureOffsetEnabledFlag").as<bool>()}
    , patchRedundancyRemoval{componentNode.require("patchRedundancyRemoval").as<bool>()}
    , numGroups{componentNode.require("numGroups").as<uint8_t>()}
    , maxEntityId{componentNode.require("maxEntityId").as<uint16_t>()}
    , viewportCameraParametersSei{componentNode.require("viewportCameraParametersSei").as<bool>()}
    , viewportPositionSei{componentNode.require("viewportPositionSei").as<bool>()}
    , codecGroupIdc{queryEnum(componentNode, "codecGroupIdc", "codec group",
                              MivBitstream::knownCodecGroupIdcs)}
    , toolsetIdc{queryEnum(componentNode, "toolsetIdc", "toolset", MivBitstream::knownToolsetIdcs)}
    , reconstructionIdc{queryEnum(componentNode, "reconstructionIdc", "reconstruction",
                                  MivBitstream::knownReconstructionIdcs)}
    , levelIdc{queryEnum(componentNode, "levelIdc", "level", MivBitstream::knownLevelIdcs)}
    , oneV3cFrameOnly{componentNode.require("oneV3cFrameOnly").as<bool>()} {
  VERIFY(0 < intraPeriod);

  if (const auto &node = componentNode.optional("interPeriod")) {
    interPeriod = node.as<int32_t>();
    VERIFY(0 < interPeriod && intraPeriod % interPeriod == 0);
  }

  queryMainParameters(componentNode);
  queryBitDepthParameters(componentNode);
  querySeiParameters(componentNode);
  queryTileParameters(componentNode);

  informationPruning = componentNode.require("informationPruning").as<bool>();

  verifyValid();
}

void Configuration::queryTileParameters(const Common::Json &componentNode) {
  if (const auto &node = componentNode.optional("partitionWidth")) {
    partitionWidth = node.asVector<int32_t>();
  }
  if (const auto &node = componentNode.optional("partitionHeight")) {
    partitionHeight = node.asVector<int32_t>();
  }
  if (partitionWidth.empty() != partitionHeight.empty()) {
    throw std::runtime_error(
        "Provide non-empty lists for both partitionWidth and partitionHeight, or provide neither.");
  }
}

void Configuration::queryMainParameters(const Common::Json &componentNode) {
  if (const auto &node = componentNode.optional("overrideAtlasFrameSizes")) {
    Common::logWarning("Overriding atlas frame sizes is meant for internal/preliminary experiments "
                       "only.");
    for (const auto &subnode : node.as<Common::Json::Array>()) {
      overrideAtlasFrameSizes.push_back(subnode.asVec<int32_t, 2>());
    }
  } else if (!oneViewPerAtlasFlag) {
    maxLumaSampleRate = componentNode.require("maxLumaSampleRate").as<double>();
    maxLumaPictureSize = componentNode.require("maxLumaPictureSize").as<int32_t>();
    maxAtlases = componentNode.require("maxAtlases").as<int32_t>();
    maxAtlases = maxAtlases / std::max(1, int32_t{numGroups});
  }

  if (haveGeometry && !haveOccupancy) {
    embeddedOccupancy = componentNode.require("embeddedOccupancy").as<bool>();
  }

  if (!haveGeometry) {
    dqParamsPresentFlag = componentNode.require("dqParamsPresentFlag").as<bool>();
  }

  if (textureOffsetFlag) {
    textureOffsetBitCount = componentNode.require("textureOffsetBitCount").as<uint32_t>();
  }

  // Read the entity encoding range if exists
  if (0 < maxEntityId) {
    entityEncRange = componentNode.require("entityEncodeRange").asVec<Common::SampleValue, 2>();
  }
}

namespace {
template <typename Idc, size_t N>
auto queryEnum(const Common::Json &node, const std::string &key, const std::string &name,
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

void Configuration::queryBitDepthParameters(const Common::Json &componentNode) {
  if (haveOccupancy) {
    occBitDepth = componentNode.require("bitDepthOccupancyVideo").as<uint32_t>();
  }

  if (haveGeometry) {
    geoBitDepth = componentNode.require("bitDepthGeometryVideo").as<uint32_t>();
  }

  if (haveTexture) {
    texBitDepth = componentNode.require("bitDepthTextureVideo").as<uint32_t>();
  }
}

void Configuration::querySeiParameters(const Common::Json &componentNode) {
  if (const auto &node = componentNode.optional("ViewingSpace")) {
    viewingSpace = MivBitstream::ViewingSpace::loadFromJson(node, componentNode);
  }
}

void Configuration::verifyValid() const {
  VERIFY(2 <= blockSize && (blockSize & (blockSize - 1)) == 0);
  VERIFY(intraPeriod <= maxIntraPeriod);
}
} // namespace TMIV::Encoder
