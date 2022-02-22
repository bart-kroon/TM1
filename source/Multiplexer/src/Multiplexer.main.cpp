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

#include <TMIV/Common/Application.h>
#include <TMIV/Decoder/DecodeV3cSampleStream.h>
#include <TMIV/IO/IO.h>
#include <TMIV/Multiplexer/DecodeAnnexBStream.h>
#include <TMIV/Multiplexer/EncodeV3cSampleStream.h>
#include <TMIV/Multiplexer/Multiplexer.h>
#include <TMIV/VideoDecoder/Partition.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>

using namespace std::string_view_literals;

namespace TMIV::Multiplexer {
class MultiplexerApplication : public Common::Application {
private:
  const std::string &m_contentId;
  int32_t m_numberOfInputFrames;
  const std::string &m_testId;
  std::ifstream m_istream;

  [[nodiscard]] auto placeholders() const {
    auto x = IO::Placeholders{};
    x.contentId = m_contentId;
    x.numberOfInputFrames = m_numberOfInputFrames;
    x.testId = m_testId;
    return x;
  }

public:
  explicit MultiplexerApplication(std::vector<const char *> argv)
      : Common::Application{"Multiplexer", std::move(argv),
                            Common::Application::Options{
                                {"-s", "Content ID (e.g. B for Museum)", false},
                                {"-n", "Number of input frames (e.g. 97)", false},
                                {"-r", "Test point (e.g. QP3 or R0)", false}}}
      , m_contentId{optionValues("-s").front()}
      , m_numberOfInputFrames{std::stoi(optionValues("-n"sv).front())}
      , m_testId{optionValues("-r").front()} {}

  void run() override {
    auto stream = openOutputBitstream();
    encodeV3cSampleStream(multiplex(openInputBitstream(), codedVideoSequenceSourceFactory()),
                          *stream);
  }

private:
  auto openInputBitstream() -> Common::Source<MivBitstream::V3cUnit> {
    const auto path = IO::inputBitstreamPath(json(), placeholders());
    m_istream.open(path, std::ios::binary);
    if (m_istream.good()) {
      return Decoder::decodeV3cSampleStream(m_istream);
    }
    throw std::runtime_error(fmt::format("Failed to open input bitstream {} for reading.", path));
  }

  auto codedVideoSequenceSourceFactory() -> CodedVideoSequenceSourceFactory {
    return [this](const MivBitstream::V3cParameterSet &vps, MivBitstream::V3cUnitHeader vuh,
                  MivBitstream::AiAttributeTypeId attrTypeId)
               -> Common::Source<std::vector<std::string>> {
      const auto path = IO::inputVideoSubBitstreamPath(json(), placeholders(), vuh, attrTypeId);
      auto stream = std::make_shared<std::ifstream>(path, std::ios::binary);
      if (stream->good()) {
        return partitionVideoSubBitstream(vps, decodeAnnexBStream(stream));
      }
      throw std::runtime_error(
          fmt::format("Failed to open input video sub-bitstream {} for reading.", path));
    };
  }

  static auto partitionVideoSubBitstream(const MivBitstream::V3cParameterSet &vps,
                                         Common::Source<std::string> source)
      -> Common::Source<std::vector<std::string>> {
    const auto codecGroupIdc = vps.profile_tier_level().ptl_profile_codec_group_idc();

    auto result = [codecGroupIdc, &source]() {
      switch (codecGroupIdc) {
      case MivBitstream::PtlProfileCodecGroupIdc::AVC_Progressive_High:
        return VideoDecoder::partitionAvcProgressiveHigh(std::move(source));
      case MivBitstream::PtlProfileCodecGroupIdc::HEVC_Main10:
        return VideoDecoder::partitionHevcMain10(std::move(source));
      case MivBitstream::PtlProfileCodecGroupIdc::HEVC444:
        return VideoDecoder::partitionHevc444(std::move(source));
      case MivBitstream::PtlProfileCodecGroupIdc::VVC_Main10:
        return VideoDecoder::partitionVvcMain10(std::move(source));
      case MivBitstream::PtlProfileCodecGroupIdc::MP4RA:
        throw std::runtime_error(fmt::format("Codec group IDC {} is not supported", codecGroupIdc));
      default:
        throw std::runtime_error(fmt::format("Unknown codec group IDC {}", codecGroupIdc));
      }
    }();
    if (result) {
      return result;
    }
    throw std::runtime_error(
        fmt::format("No built-in support for the {} codec group IDC", codecGroupIdc));
  }

  auto openOutputBitstream() -> std::unique_ptr<std::ostream> {
    const auto path = IO::outputBitstreamPath(json(), placeholders());
    auto stream = std::make_unique<std::ofstream>(path, std::ios::binary);
    if (!stream->good()) {
      throw std::runtime_error(fmt::format("Failed to open {} for writing", path));
    }
    return stream;
  }
};
} // namespace TMIV::Multiplexer

auto main(int argc, char *argv[]) -> int32_t {
  try {
    TMIV::Multiplexer::MultiplexerApplication app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (std::runtime_error &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
