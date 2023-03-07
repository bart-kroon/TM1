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

#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/Decoder/DecodeAtlasSubBitstream.h>
#include <TMIV/Decoder/DecodeMiv.h>
#include <TMIV/Decoder/DecodeV3cSampleStream.h>
#include <TMIV/MivBitstream/Formatters.h>
#include <TMIV/PtlChecker/PtlChecker.h>

#include "MockVideoDecoder.hpp"

#include <fstream>

using namespace std::string_view_literals;

namespace TMIV::Decoder::test {
// Modified PTL checker that does not inspect the video frame
class ModifiedPtlChecker final : public PtlChecker::PtlChecker {
public:
  // Skip the video frame check
  void checkVideoFrame([[maybe_unused]] MivBitstream::VuhUnitType vut,
                       [[maybe_unused]] const MivBitstream::AtlasSequenceParameterSetRBSP &asps,
                       [[maybe_unused]] const Common::Frame<> &frame) final {}
};

// This integration test verifies that the MIV decoder can be run with a video decoder that does
// not return a frame, for instance because the decoding happens on the GPU.
class NativeVideoDecoderTest {
private:
  std::istream &m_stream;
  int32_t m_frameCount;
  int32_t m_intraPeriod;
  PtlChecker::SharedChecker m_checker;
  Common::Source<MivBitstream::AccessUnit> m_mivDecoder;

public:
  NativeVideoDecoderTest(std::istream &stream, int32_t frameCount, int32_t intraPeriod)
      : m_stream{stream}
      , m_frameCount{frameCount}
      , m_intraPeriod{intraPeriod}
      , m_checker{std::make_shared<ModifiedPtlChecker>()}
      , m_mivDecoder{decodeMiv()} {}

  void run() {
    while (auto frame = m_mivDecoder()) {
      ;
    }
  }

private:
  auto decodeMiv() -> Common::Source<MivBitstream::AccessUnit> {
    return Decoder::decodeMiv(decodeV3cSampleStream(m_stream), videoDecoderFactory(), m_checker,
                              commonAtlasDecoderFactory(), atlasDecoderFactory());
  }

  [[nodiscard]] auto videoDecoderFactory() const -> VideoDecoderFactory {
    return [this](Common::Source<MivBitstream::VideoSubBitstream> source,
                  [[maybe_unused]] const MivBitstream::V3cParameterSet &vps,
                  [[maybe_unused]] MivBitstream::V3cUnitHeader vuh)
               -> Common::Source<Common::DecodedFrame> {
      return MockVideoDecoder{std::move(source), m_frameCount, m_intraPeriod};
    };
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
} // namespace TMIV::Decoder::test

using TMIV::Common::logError;
using TMIV::Common::logInfo;

auto main(int argc, char *argv[]) -> int32_t {
  try {
    const auto args = std::vector(argv, argv + argc);

    if (argc != 4) {
      logInfo("Usage: TmivNativeVideoDecoderTest BITSTREAM FRAME_COUNT INTRA_PERIOD");
      logInfo("");
      logInfo("  * The BITSTREAM should be a V3C sample stream with a supported profile.");
      logInfo("  * The FRAME_COUNT and INTRA_PERIOD parameters must match the BITSTREAM,");
      logInfo("    otherwise the test is invalid.");
      return 1;
    }

    std::ifstream stream{args[1], std::ios::binary};

    if (!stream.good()) {
      logError("Failed to open {} for reading.", args[1]);
      return 1;
    }

    auto test =
        TMIV::Decoder::test::NativeVideoDecoderTest{stream, std::atoi(args[2]), std::atoi(args[3])};
    test.run();
    return 0;
  } catch (...) {
    return TMIV::Common::handleException();
  }
}
