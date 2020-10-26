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

#include <TMIV/Renderer/mapInputToOutputFrames.h>

namespace TMIV::Renderer {
namespace {
// Returns a frame index. If frameIndex is strictly less than the actual number of frames in the
// encoded stream, then regular values are returned else mirrored indices are computed.
auto getExtendedIndex(const Common::Json &config, int frameIndex) -> int {
  int numberOfFrames = config.require("numberOfFrames").as<int>();
  int frameGroupId = frameIndex / numberOfFrames;
  int frameRelativeId = frameIndex % numberOfFrames;
  return (frameGroupId % 2) != 0 ? (numberOfFrames - (frameRelativeId + 1)) : frameRelativeId;
}
} // namespace

auto mapInputToOutputFrames(const Common::Json &config) -> std::multimap<int, int> {
  auto x = std::multimap<int, int>{};

  const auto numberOfFrames = config.require("numberOfFrames").as<int>();
  auto extraFrames = 0;
  auto firstOutputFrame = 0;
  auto outputFrameStep = 1;

  if (const auto &subnode = config.optional("extraNumberOfFrames")) {
    extraFrames = subnode.as<int>();
  }

  if (const auto &subnode = config.optional("firstOutputFrame")) {
    firstOutputFrame = subnode.as<int>();
  }

  if (const auto &subnode = config.optional("outputFrameStep")) {
    outputFrameStep = subnode.as<int>();
  }

  for (int outputFrame = firstOutputFrame; outputFrame < numberOfFrames + extraFrames;
       outputFrame += outputFrameStep) {
    const auto inputFrame = getExtendedIndex(config, outputFrame);
    x.emplace(inputFrame, outputFrame);
  }

  return x;
}
} // namespace TMIV::Renderer