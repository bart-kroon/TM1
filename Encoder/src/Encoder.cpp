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

#include <TMIV/Encoder/Encoder.h>

#include <TMIV/Common/Factory.h>

#include <cassert>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

// Encoder sub-component interfaces
using TMIV::Aggregator::IAggregator;
using TMIV::GeometryQuantizer::IGeometryQuantizer;
using TMIV::Packer::IPacker;
using TMIV::Pruner::IPruner;
using TMIV::ViewOptimizer::IViewOptimizer;

namespace TMIV::Encoder {
namespace {
template <typename Interface>
auto create(const char *name, const Json &rootNode, const Json &componentNode) {
  const auto &instance = Factory<Interface>::getInstance();
  return instance.create(name, rootNode, componentNode);
}

void runtimeCheck(bool cond, const char *what) {
  if (!cond) {
    throw runtime_error(what);
  }
}
} // namespace

Encoder::Encoder(const Json &rootNode, const Json &componentNode)
    : m_viewOptimizer{create<IViewOptimizer>("ViewOptimizer", rootNode, componentNode)}
    , m_pruner{create<Pruner::IPruner>("Pruner", rootNode, componentNode)}
    , m_aggregator{create<IAggregator>("Aggregator", rootNode, componentNode)}
    , m_packer{create<IPacker>("Packer", rootNode, componentNode)}
    , m_depthOccupancy{create<IGeometryQuantizer>("GeometryQuantizer", rootNode, componentNode)}
    , m_geometryDownscaler{rootNode, componentNode} {
  // Parameters
  const auto numGroups = rootNode.require("numGroups").asInt();
  m_blockSize = rootNode.require("blockSize").asInt();
  const auto maxLumaSampleRate = rootNode.require("maxLumaSampleRate").asDouble();
  const auto maxLumaPictureSize = rootNode.require("maxLumaPictureSize").asInt();
  const auto maxAtlases = rootNode.require("maxAtlases").asInt();
  m_geometryScaleEnabledFlag = rootNode.require("geometryScaleEnabledFlag").asBool();

  if (auto node = componentNode.optional("dilate"); node) {
    m_dilationIter = node.asInt();
  }

  if (auto node = componentNode.optional("overrideAtlasFrameSizes"); node) {
    cout << "WARNING: Overriding atlas frame sizes is meant for internal/preliminary experiments "
            "only.\n";
    for (size_t i = 0; i < node.size(); ++i) {
      m_overrideAtlasFrameSizes.push_back(node.at(i).asIntVector<2>());
    }
  }

  // Check parameters
  runtimeCheck(1 <= numGroups, "numGroups should be at least one");
  runtimeCheck(2 <= m_blockSize, "blockSize should be at least two");
  runtimeCheck((m_blockSize & (m_blockSize - 1)) == 0, "blockSize should be a power of two");
  if (maxLumaSampleRate == 0) {
    runtimeCheck(maxLumaPictureSize == 0 && maxAtlases == 0,
                 "Either specify all constraints or none");
  } else {
    runtimeCheck(maxLumaPictureSize > 0 && maxAtlases > 0,
                 "Either specify all constraints or none");
    runtimeCheck(numGroups <= maxAtlases, "There should be at least one atlas per group");
  }

  // Translate parameters to concrete constraints
  const auto lumaSamplesPerAtlasSample = m_geometryScaleEnabledFlag ? 1.25 : 2.;
  m_maxBlockRate = maxLumaSampleRate / (numGroups * lumaSamplesPerAtlasSample * sqr(m_blockSize));
  m_maxBlocksPerAtlas = maxLumaPictureSize / sqr(m_blockSize);
  m_maxAtlases = maxAtlases / numGroups;

  // Read the entity encoding range if exisited
  if (auto subnode = componentNode.optional("EntityEncodeRange")) {
    m_entityEncRange = subnode.asIntVector<2>();
  }

  if (rootNode.require("intraPeriod").asInt() > maxIntraPeriod) {
    throw runtime_error("The intraPeriod parameter cannot be greater than maxIntraPeriod.");
  }

  // Check if running in explicit occupancy coding mode
  if ((componentNode.require("GeometryQuantizerMethod").asString() == "ExplicitOccupancy")) {
    m_ExternalOccupancyCoding = true;
  }
}

/*
auto Encoder::prepareSequence(MivBitstream::IvSequenceParams ivSequenceParams)
    -> const MivBitstream::IvSequenceParams & {
  auto optimal = m_viewOptimizer->optimizeSequence(move(ivSequenceParams));
  int index = 0;
  for (auto i = 0; i < optimal.second.size(); i++)
    if (optimal.second[i]) {
      optimal.first.vps.vps_miv_extension().vme_fully_occupied_flag(
          index, true); // assuming atlas size is equal to view size and basic views
                        // are packed in atlases first
      optimal.first.vps.vps_miv_extension().vme_occupancy_subbitstream_present_flag(index, false);
      index++;
    }
  return m_geometryDownscaler.transformSequenceParams(m_depthOccupancy->transformSequenceParams(
      m_atlasConstructor->prepareSequence(move(optimal.first), move(optimal.second)))); // ToDo Basel
}
*/

auto Encoder::maxLumaSamplesPerFrame() const -> size_t { return m_maxLumaSamplesPerFrame; }
} // namespace TMIV::Encoder
