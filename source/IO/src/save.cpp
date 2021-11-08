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

#include <TMIV/IO/IO.h>

#include <fstream>

using namespace std::string_literals;

namespace TMIV::IO {
template <typename Element>
void saveFrame(const std::filesystem::path &path, const Common::Frame<Element> &frame,
               int32_t frameIdx) {
  Common::withElement(frame.getBitDepth(), [&](auto zero) {
    using NativeElement = decltype(zero);

    if constexpr (std::is_same_v<NativeElement, Element>) {
      create_directories(path.parent_path());

      std::fstream stream(path, frameIdx == 0 ? std::ios::out | std::ios::binary
                                              : std::ios::in | std::ios::out | std::ios::binary);
      if (!stream.good()) {
        throw std::runtime_error(fmt::format("Failed to open {} for writing", path));
      }

      stream.seekp(int64_t{frameIdx} * frame.getByteCount());
      if (!stream.good()) {
        throw std::runtime_error(
            fmt::format("Failed to seek for writing to frame {} of {}", frameIdx, path));
      }

      frame.writeTo(stream);

      if (!stream.good()) {
        throw std::runtime_error(fmt::format("Failed to write to {}", path));
      }
    } else {
      saveFrame(path, Common::elementCast<NativeElement>(frame), frameIdx);
    }
  });
}

void saveOutOfBandVideoFrame(const Common::Json &config, const Placeholders &placeholders,
                             const Common::Frame<> &frame, MivBitstream::V3cUnitHeader vuh,
                             int32_t frameIdx, MivBitstream::AiAttributeTypeId attrTypeId) {
  PRECONDITION(!frame.empty());

  const auto outputDir = config.require("outputDirectory").as<std::filesystem::path>();

  const auto configKey =
      fmt::format("output{}VideoDataPathFmt", videoComponentName(vuh.vuh_unit_type(), attrTypeId));

  const auto path =
      outputDir / fmt::format(config.require(configKey).as<std::string>(),
                              placeholders.numberOfInputFrames, placeholders.contentId,
                              placeholders.testId, vuh.vuh_atlas_id(), frame.getWidth(),
                              frame.getHeight(), videoFormatString(frame));

  saveFrame(path, frame, frameIdx);
}

void saveViewport(const Common::Json &config, const Placeholders &placeholders, int32_t frameIdx,
                  const std::string &name, const Common::DeepFrame &frame) {
  const auto outputDir = config.require("outputDirectory").as<std::filesystem::path>();
  auto saved = false;

  if (const auto &node = config.optional("outputViewportTexturePathFmt")) {
    const auto frame_ = yuv420(frame.texture);
    saveFrame(outputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                      placeholders.contentId, placeholders.testId,
                                      placeholders.numberOfOutputFrames, name,
                                      frame.texture.getWidth(), frame.texture.getHeight(),
                                      videoFormatString(frame_)),
              frame_, frameIdx);
    saved = true;
  }
  if (const auto &node = config.optional("outputViewportGeometryPathFmt")) {
    const auto frame_ = yuv420(frame.geometry);
    saveFrame(outputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                      placeholders.contentId, placeholders.testId,
                                      placeholders.numberOfOutputFrames, name,
                                      frame.geometry.getWidth(), frame.geometry.getHeight(),
                                      videoFormatString(frame_)),
              frame_, frameIdx);
    saved = true;
  }

  if (!saved) {
    fmt::print("WARNING: Calculated viewport but not saving texture or geometry. Add "
               "outputViewportTexturePathFmt or outputViewportGeometryPathFmt to "
               "the configuration file.\n");
  }
}

void optionalSaveBlockToPatchMaps(const Common::Json &config, const Placeholders &placeholders,
                                  int32_t frameIdx, const MivBitstream::AccessUnit &frame) {
  const auto outputDir = config.require("outputDirectory").as<std::filesystem::path>();

  if (const auto &node = config.optional("outputBlockToPatchMapPathFmt")) {
    for (size_t k = 0; k < frame.atlas.size(); ++k) {
      const auto &btpm = frame.atlas[k].blockToPatchMap;
      saveFrame(outputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                        placeholders.contentId, placeholders.testId, k,
                                        btpm.getWidth(), btpm.getHeight(), videoFormatString(btpm)),
                btpm, frameIdx);
    }
  }
}

void optionalSavePrunedFrame(const Common::Json &config, const Placeholders &placeholders,
                             const Common::Frame<> &frame, MivBitstream::VuhUnitType vut,
                             std::pair<int32_t, uint16_t> frameViewIdx,
                             MivBitstream::AiAttributeTypeId attrTypeId) {
  if (frame.empty()) {
    return;
  }

  const auto configKey =
      fmt::format("outputMultiview{}PathFmt", videoComponentName(vut, attrTypeId));

  if (const auto &node = config.optional(configKey)) {
    const auto outputDir = config.require("outputDirectory").as<std::filesystem::path>();
    saveFrame(outputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                      placeholders.contentId, placeholders.testId,
                                      frameViewIdx.second, frame.getWidth(), frame.getHeight(),
                                      videoFormatString(frame)),
              frame, frameViewIdx.first);
  }
}

void optionalSaveSequenceConfig(const Common::Json &config, const Placeholders &placeholders,
                                int32_t foc, const MivBitstream::SequenceConfig &seqConfig) {
  if (const auto &node = config.optional("outputSequenceConfigPathFmt")) {
    const auto path = config.require("outputDirectory").as<std::filesystem::path>() /
                      fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                  placeholders.contentId, placeholders.testId, foc);

    // NOTE(#483): Binary mode to prevent problems with cross-platform consistency checks
    std::ofstream stream{path, std::ios::binary};
    const auto json = Common::Json{seqConfig};
    json.saveTo(stream);
  }
}

auto outputBitstreamPath(const Common::Json &config, const Placeholders &placeholders)
    -> std::filesystem::path {
  auto path =
      config.require("outputDirectory").as<std::filesystem::path>() /
      fmt::format(config.require("outputBitstreamPathFmt").as<std::string>(),
                  placeholders.numberOfInputFrames, placeholders.contentId, placeholders.testId);
  create_directories(path.parent_path());
  return path;
}
} // namespace TMIV::IO
