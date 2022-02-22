/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#ifndef TMIV_IO_IO_H
#define TMIV_IO_IO_H

#include <TMIV/Common/Frame.h>
#include <TMIV/Common/Json.h>
#include <TMIV/MivBitstream/AccessUnit.h>

namespace TMIV::IO {
struct Placeholders {
  std::string contentId{};        // e.g. A
  std::string testId{"R0"};       // e.g. QP3 or R0
  int32_t numberOfInputFrames{};  // e.g. 97
  int32_t numberOfOutputFrames{}; // e.g. 300
  int32_t startFrame{};           // e.g. 23
};

auto loadMultiviewFrame(const Common::Json &config, const Placeholders &placeholders,
                        const MivBitstream::SequenceConfig &sc, int32_t frameIdx)
    -> Common::DeepFrameList;

auto loadViewportMetadata(const Common::Json &config, const Placeholders &placeholders,
                          int32_t frameIdx, const std::string &cameraName, bool isPoseTrace)
    -> MivBitstream::CameraConfig;

auto loadOutOfBandVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                             MivBitstream::V3cUnitHeader vuh, int32_t frameIdx)
    -> Common::DecodedFrame;

auto loadSequenceConfig(const Common::Json &config, const Placeholders &placeholders,
                        int32_t frameIdx) -> MivBitstream::SequenceConfig;

auto tryLoadSequenceConfig(const Common::Json &config, const Placeholders &placeholders,
                           int32_t frameIdx) -> std::optional<MivBitstream::SequenceConfig>;

auto loadMpiTextureMpiLayer(const Common::Json &config, const Placeholders &placeholders,
                            const MivBitstream::SequenceConfig &sc, int32_t frameIdx,
                            int32_t mpiLayerIdx, int32_t nbMpiLayers) -> Common::Frame<>;

auto loadMpiTransparencyMpiLayer(const Common::Json &config, const Placeholders &placeholders,
                                 const MivBitstream::SequenceConfig &sc, int32_t frameIdx,
                                 int32_t mpiLayerIdx, int32_t nbMpiLayers) -> Common::Frame<>;

[[nodiscard]] auto inputBitstreamPath(const Common::Json &config, const Placeholders &placeholders)
    -> std::filesystem::path;

[[nodiscard]] auto inputVideoSubBitstreamPath(
    const Common::Json &config, const Placeholders &placeholders, MivBitstream::V3cUnitHeader vuh,
    MivBitstream::AiAttributeTypeId attrTypeId = MivBitstream::AiAttributeTypeId::ATTR_UNSPECIFIED)
    -> std::filesystem::path;

void saveOutOfBandMetadata(const Common::Json &config, const Placeholders &placeholders,
                           Common::Json::Array metadata);

auto saveOutOfBandVideoFrame(
    const Common::Json &config, const Placeholders &placeholders, const Common::Frame<> &frame,
    MivBitstream::V3cUnitHeader vuh, int32_t frameIdx,
    MivBitstream::AiAttributeTypeId attrTypeId = MivBitstream::AiAttributeTypeId::ATTR_UNSPECIFIED)
    -> Common::Json::Object;

void saveViewport(const Common::Json &config, const Placeholders &placeholders, int32_t frameIdx,
                  const std::string &name, const Common::DeepFrame &frame);

void optionalSaveBlockToPatchMaps(const Common::Json &config, const Placeholders &placeholders,
                                  int32_t frameIdx, const MivBitstream::AccessUnit &frame);

void optionalSavePrunedFrame(
    const Common::Json &config, const Placeholders &placeholders, const Common::Frame<> &frame,
    MivBitstream::VuhUnitType vut, std::pair<int32_t, uint16_t> frameViewIdx,
    MivBitstream::AiAttributeTypeId attrTypeId = MivBitstream::AiAttributeTypeId::ATTR_UNSPECIFIED);

void optionalSaveSequenceConfig(const Common::Json &config, const Placeholders &placeholders,
                                int32_t frameIdx, const MivBitstream::SequenceConfig &seqConfig);

// Construct the output bitstream path and create the parent directories
auto outputBitstreamPath(const Common::Json &config, const Placeholders &placeholders)
    -> std::filesystem::path;

auto videoComponentName(MivBitstream::VuhUnitType vuhUnitType,
                        MivBitstream::AiAttributeTypeId attrTypeId =
                            MivBitstream::AiAttributeTypeId::ATTR_UNSPECIFIED) -> std::string;

auto videoFormatString(Common::ColorFormat colorFormat, uint32_t bitDepth) -> std::string;

template <typename Element>
auto videoFormatString(const Common::Frame<Element> &frame) -> std::string {
  return videoFormatString(frame.getColorFormat(), frame.getBitDepth());
}
} // namespace TMIV::IO

#endif
