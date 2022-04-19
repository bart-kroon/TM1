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

#include <TMIV/Decoder/DecodeAtlasSubBitstream.h>
#include <TMIV/Decoder/DecodeMiv.h>
#include <TMIV/Decoder/DecodeNalUnitStream.h>
#include <TMIV/Decoder/DecodeV3cSampleStream.h>
#include <TMIV/Decoder/DecodeVideoSubBitstream.h>
#include <TMIV/Decoder/OutputLog.h>
#include <TMIV/PtlChecker/PtlChecker.h>
#include <TMIV/VideoDecoder/VideoDecoder.h>

#include <fmt/ostream.h>

#include <fstream>

using namespace std::string_view_literals;
using namespace std::string_literals;

namespace TMIV::Decoder {
class Application {
private:
  std::istream &m_inputBitstream;
  std::ostream &m_outputLog;
  PtlChecker::SharedChecker m_checker;
  Common::Source<MivBitstream::AccessUnit> m_mivDecoder;

public:
  explicit Application(std::istream &inputBitstream, std::ostream &outputLog)
      : m_inputBitstream{inputBitstream}
      , m_outputLog{outputLog}
      , m_checker{std::make_shared<PtlChecker::PtlChecker>()}
      , m_mivDecoder{decodeMiv()} {
    // NOTE(BK): PTL warnings are errors because there is no value in having a decoder output log
    // for a non-conformant bitstream and otherwise the PTL warning may go unnoticed.
    m_checker->replaceLogger([](const std::string &message) {
      throw std::runtime_error(
          fmt::format("ERROR: A profile-tier-level check has failed: {}", message));
    });
  }

  void run() {
    while (auto frame = m_mivDecoder()) {
      writeFrameToOutputLog(*frame, m_outputLog);
    }
  }

private:
  auto decodeMiv() -> Common::Source<MivBitstream::AccessUnit> {
    return Decoder::decodeMiv(decodeV3cSampleStream(m_inputBitstream), videoDecoderFactory(),
                              m_checker, commonAtlasDecoderFactory(), atlasDecoderFactory());
  }

  static auto videoDecoderFactory() -> VideoDecoderFactory {
    return [](Common::Source<MivBitstream::VideoSubBitstream> source,
              const MivBitstream::V3cParameterSet &vps,
              [[maybe_unused]] MivBitstream::V3cUnitHeader vuh)
               -> Common::Source<Common::DecodedFrame> {
      return decodeVideo(vps, decodeNalUnitStream(decodeVideoSubBitstream(std::move(source))));
    };
  }

  static auto decodeVideo(const MivBitstream::V3cParameterSet &vps,
                          Common::Source<std::string> source)
      -> Common::Source<Common::DecodedFrame> {
    const auto codecGroupIdc = vps.profile_tier_level().ptl_profile_codec_group_idc();

    auto result = [codecGroupIdc, &source]() {
      switch (codecGroupIdc) {
      case MivBitstream::PtlProfileCodecGroupIdc::AVC_Progressive_High:
        return VideoDecoder::decodeAvcProgressiveHigh(std::move(source));
      case MivBitstream::PtlProfileCodecGroupIdc::HEVC_Main10:
        return VideoDecoder::decodeHevcMain10(std::move(source));
      case MivBitstream::PtlProfileCodecGroupIdc::HEVC444:
        return VideoDecoder::decodeHevc444(std::move(source));
      case MivBitstream::PtlProfileCodecGroupIdc::VVC_Main10:
        return VideoDecoder::decodeVvcMain10(std::move(source));
      default:
        return Common::Source<Common::DecodedFrame>{};
      }
    }();

    if (result) {
      return result;
    }
    throw std::runtime_error(
        fmt::format("Failed to initialize a video decoder for codec group IDC {}", codecGroupIdc));
  }

  auto commonAtlasDecoderFactory() -> CommonAtlasDecoderFactory {
    return [checker = m_checker](
               Common::Source<MivBitstream::AtlasSubBitstream> source,
               const MivBitstream::V3cParameterSet &vps) -> Common::Source<CommonAtlasAccessUnit> {
      return decodeCommonAtlas(decodeAtlasSubBitstream(std::move(source)), vps, checker);
    };
  }

  auto atlasDecoderFactory() -> AtlasDecoderFactory {
    return
        [checker = m_checker](Common::Source<MivBitstream::AtlasSubBitstream> source,
                              const MivBitstream::V3cParameterSet &vps,
                              MivBitstream::V3cUnitHeader vuh) -> Common::Source<AtlasAccessUnit> {
          return decodeAtlas(decodeAtlasSubBitstream(std::move(source)), vuh, vps, checker);
        };
  }
};
} // namespace TMIV::Decoder

auto main(int argc, char *argv[]) -> int32_t {
  try {
    const auto args = std::vector(argv, argv + argc);

    if (args.size() != 5 || args[1] != "-b"sv || args[3] != "-o"sv) {
      std::cerr << "Usage: DecoderLog -b BITSTREAM -o DECODER_LOG";
      return 1;
    }

    std::ifstream inStream{args[2], std::ios::binary};

    if (!inStream.good()) {
      fmt::print("Failed to open {} for reading.\n", args[2]);
      return 1;
    }

    std::ofstream outStream{args[4], std::ios::binary};

    if (!outStream.good()) {
      fmt::print("Failed to open {} for writing.\n", args[4]);
      return 1;
    }

    auto decoderLog = TMIV::Decoder::Application{inStream, outStream};
    decoderLog.run();
    return 0;
  } catch (std::runtime_error &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  } catch (std::bad_function_call &e) {
    std::cerr << e.what() << std::endl;
    return 2;
  } catch (std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 3;
  }
}
