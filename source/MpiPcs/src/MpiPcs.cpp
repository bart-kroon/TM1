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

#include <TMIV/MpiPcs/MpiPcs.h>

#include <fstream>

namespace TMIV::MpiPcs {
namespace {
template <typename T>
auto readFromStream(std::istream &stream, uint64_t numberOfItems) -> std::vector<T> {
  std::string itemBuffer(numberOfItems * sizeof(T), '0');
  stream.read(itemBuffer.data(), Common::downCast<std::streamsize>(itemBuffer.size()));
  if (!stream.good()) {
    throw std::runtime_error("Failed to read stream");
  }

  std::vector<T> items(numberOfItems);
  memcpy(items.data(), itemBuffer.data(), itemBuffer.size());
  return items;
}
} // namespace

const std::string inputMpiPcsPathFmt = "inputMpiPcsPathFmt";
const std::string outputMpiPcsPathFmt = "outputMpiPcsPathFmt";

void FileHeader::read(std::istream &stream) {
  std::array<char, content.size()> buffer{};

  stream.read(buffer.data(), buffer.size());
  if (!stream.good()) {
    throw std::runtime_error("Failed to read from stream");
  }

  if (buffer != content) {
    throw std::runtime_error("MpiPcs file header mismatch");
  }
}

void FileHeader::write(std::ostream &stream) {
  stream.write(content.data(), content.size());
  if (!stream.good()) {
    throw std::runtime_error("Failed to write to stream");
  }
}

Reader::Reader(const Common::Json &config, const IO::Placeholders &placeholders,
               const MivBitstream::SequenceConfig &sc, const bool buildIndexOn)
    : m_startFrame{placeholders.startFrame} {
  const auto inputDir = config.require("inputDirectory").as<std::filesystem::path>();
  const auto &node = config.require(inputMpiPcsPathFmt);

  const auto cameraName = sc.sourceCameraNames[0];

  const auto cameraConfig = sc.cameraByName(cameraName);
  const auto videoFormat = IO::videoFormatString(cameraConfig.colorFormatTransparency,
                                                 cameraConfig.bitDepthTransparency);

  m_size = cameraConfig.viewParams.ci.projectionPlaneSize();
  m_path = inputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                  placeholders.contentId, placeholders.testId, cameraName,
                                  m_size.x(), m_size.y(), videoFormat);

  if (buildIndexOn) {
    buildIndex();
  }
}

auto Reader::read(std::istream &stream, std::streampos posId, Common::Vec2i size) -> Frame {
  stream.seekg(posId);
  if (!stream.good()) {
    throw std::runtime_error(fmt::format("Failed to seek stream at position {}", posId));
  }

  std::vector<Pixel> pixelList(size.x() * size.y());

  const auto countList = readFromStream<Attribute::Count>(stream, pixelList.size());

  const auto numberOfAttributes = std::accumulate(countList.begin(), countList.end(), 0ULL);

  const auto bufferList = readFromStream<Attribute::Buffer>(stream, numberOfAttributes);

  for (auto k = 0ULL, l = 0ULL; k < pixelList.size(); ++k) {
    auto &pixel = pixelList[k];

    pixel.reserve(countList[k]);

    for (Attribute::Count i = 0; i < countList[k]; ++i, ++l) {
      pixel.push_back(Attribute::fromBuffer(bufferList[l]));
    }
  }

  return Frame{m_size, std::move(pixelList)};
}

auto Reader::read(int32_t frameIdx) -> Frame {
  std::ifstream stream{m_path, std::ifstream::binary};
  if (!stream.good()) {
    throw std::runtime_error(fmt::format("Failed to open {} for reading", m_path));
  }

  fmt::print("Loading MPI pcs frame {0} with start frame offset {1} (= {2}).\n", frameIdx,
             m_startFrame, frameIdx + m_startFrame);

  return read(stream, m_index[frameIdx + m_startFrame], m_size);
}

void Reader::buildIndex() {
  std::ifstream stream{m_path, std::ifstream::binary};
  if (!stream.good()) {
    throw std::runtime_error(fmt::format("Failed to open {} for reading", m_path));
  }

  stream.seekg(0, std::ifstream::end);
  if (!stream.good()) {
    throw std::runtime_error(fmt::format("Failed to seekg from {}", m_path));
  }

  const auto length = stream.tellg();

  stream.seekg(0, std::ifstream::beg);
  if (!stream.good()) {
    throw std::runtime_error(fmt::format("Failed to seekg from {}", m_path));
  }

  FileHeader::read(stream);

  std::vector<Attribute::Count> countList(m_size.x() * m_size.y());

  auto pos = stream.tellg();

  while (pos < length) {
    m_index.emplace_back(pos);

    stream.seekg(pos);
    if (!stream.good()) {
      throw std::runtime_error(fmt::format("Failed to seekg from {}", m_path));
    }

    std::string attributeCountBuffer(countList.size() * sizeof(Attribute::Count), '0');

    stream.read(attributeCountBuffer.data(),
                Common::downCast<std::streamsize>(attributeCountBuffer.size()));
    if (!stream.good()) {
      throw std::runtime_error(fmt::format("Failed to read from {}", m_path));
    }

    std::memcpy(countList.data(), attributeCountBuffer.data(), attributeCountBuffer.size());

    const auto nbAttribute = std::accumulate(countList.begin(), countList.end(), 0ULL);

    pos += Common::downCast<std::streamoff>(countList.size() * sizeof(Attribute::Count) +
                                            nbAttribute * Attribute::attributeSize);
  }
}

Writer::Writer(const Common::Json &config, const IO::Placeholders &placeholders,
               const MivBitstream::SequenceConfig &sc) {
  const auto outputDir = config.require("outputDirectory").as<std::filesystem::path>();
  const auto &node = config.require(outputMpiPcsPathFmt);

  const auto cameraName = sc.sourceCameraNames[0];

  const auto cameraConfig = sc.cameraByName(cameraName);
  const auto videoFormat = IO::videoFormatString(cameraConfig.colorFormatTransparency,
                                                 cameraConfig.bitDepthTransparency);

  const auto size = cameraConfig.viewParams.ci.projectionPlaneSize();
  m_path = outputDir / fmt::format(node.as<std::string>(), placeholders.numberOfInputFrames,
                                   placeholders.contentId, placeholders.testId, cameraName,
                                   size.x(), size.y(), videoFormat);

  create_directories(m_path.parent_path());

  std::ofstream stream{m_path, std::ofstream::binary};
  if (!stream.good()) {
    throw std::runtime_error(fmt::format("Failed to open {} for writing", m_path));
  }

  FileHeader::write(stream);
}

void Writer::append(std::ostream &stream, const Frame &mpiPcsFrame) {
  std::vector<Attribute::Count> counts;
  std::vector<Attribute::Buffer> buffers;

  for (const auto &pixel : mpiPcsFrame.getPixelList()) {
    counts.emplace_back(static_cast<Attribute::Count>(pixel.size()));

    for (const auto &attribute : pixel) {
      buffers.emplace_back(attribute.toBuffer());
    }
  }

  writeToStream(stream, counts);
  writeToStream(stream, buffers);
}

template <typename T>
void Writer::writeToStream(std::ostream &stream, std::vector<T> &items) const {
  std::string itemBuffer(items.size() * sizeof(T), '0');
  memcpy(itemBuffer.data(), items.data(), itemBuffer.size());

  stream.write(itemBuffer.data(), Common::downCast<std::streamsize>(itemBuffer.size()));
  if (!stream.good()) {
    throw std::runtime_error(fmt::format("Failed to write to {}", m_path));
  }
}

void Writer::append(const Frame &mpiPcsFrame) {
  std::ofstream stream{m_path, std::ofstream::binary | std::ofstream::app};
  if (!stream.good()) {
    throw std::runtime_error(fmt::format("Failed to open {} for appending", m_path));
  }

  append(stream, mpiPcsFrame);
}
} // namespace TMIV::MpiPcs
