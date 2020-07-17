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

#ifndef _TMIV_IO_IO_H_
#error "Include the .h, not the .hpp"
#endif

#include <TMIV/Common/Common.h>

#include <cstring>
#include <fstream>
#include <iostream>

namespace TMIV::IO {
template <typename... Args>
auto getFullPath(const Common::Json &config, const std::string &baseDirectoryField,
                 const std::string &fileNameField, Args &&... args) -> std::string {
  std::string baseDirectory;
  auto fileName = Common::format(config.require(fileNameField).asString(), args...);

  // Detect absolute paths for /POSIX, \Windows and C:\Windows
  if ((!fileName.empty() && (fileName.front() == '/' || fileName.front() == '\\')) ||
      (fileName.size() >= 2 && fileName[1] == ':')) {
    return fileName;
  }

  if (auto subnode = config.optional(baseDirectoryField)) {
    baseDirectory = subnode.asString() + "/";
  }

  return baseDirectory + fileName;
}

template <typename FORMAT, typename... Args>
auto readFrame(const Common::Json &config, const std::string &baseDirectoryField,
               const std::string &fileNameField, int frameIndex, Common::Vec2i resolution,
               Args &&... args) -> Common::Frame<FORMAT> {
  auto result = Common::Frame<FORMAT>(resolution.x(), resolution.y());

  const auto path = getFullPath(config, baseDirectoryField, fileNameField,
                                std::forward<Args>(args)..., resolution.x(), resolution.y());

  std::cout << "Reading frame " << frameIndex << " from " << path << '\n';
  std::ifstream stream{path, std::ios::binary};

  if (!stream.good()) {
    if (fileNameField == "OccupancyVideoDataPathFmt") {
      std::cout << "No occupancy info, either the file is corrupted or atlas is fully occupied or occupancy is embedded in geometry\n";
      return result; // Handling the case of embeded occupancy since not known ahead (occupancy
                     // signaling elements are not read yet)
    }
    throw std::runtime_error("Failed to open file");
  }

  stream.seekg(std::streampos(frameIndex) * result.getDiskSize());
  result.read(stream);

  if (!stream.good()) {
    return Common::Frame<FORMAT>{};
  }

  return result;
}

namespace {
template <typename FORMAT> void padChroma(std::ostream &stream, int bytes) {
  constexpr auto fillValue = Common::neutralColor<FORMAT>();
  const auto padding = std::vector(bytes / sizeof(fillValue), fillValue);
  auto buffer = std::vector<char>(bytes);
  std::memcpy(buffer.data(), padding.data(), buffer.size());
  stream.write(buffer.data(), buffer.size());
}
} // namespace

template <typename FORMAT, typename... Args>
void writeFrame(const Common::Json &config, const std::string &fileNameField,
                const Common::Frame<FORMAT> &frame, int frameIndex, Args &&... args) {
  const auto path = getFullPath(config, "OutputDirectory", fileNameField,
                                std::forward<Args>(args)..., frame.getWidth(), frame.getHeight());
  std::cout << "Writing frame " << frameIndex << " to " << path << '\n';

  std::fstream stream(path, frameIndex == 0 ? std::ios::out | std::ios::binary
                                            : std::ios::in | std::ios::out | std::ios::binary);
  if (!stream.good()) {
    throw std::runtime_error("Failed to open file for writing");
  }

  stream.seekp(int64_t(frameIndex) * frame.getDiskSize());

  frame.dump(stream);
  padChroma<FORMAT>(stream, frame.getDiskSize() - frame.getMemorySize());

  if (!stream.good()) {
    throw std::runtime_error("Failed to write to file");
  }
}
} // namespace TMIV::IO