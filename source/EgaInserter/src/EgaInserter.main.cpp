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
 * int32_tERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <TMIV/EgaInserter/EgaInserter.h>

#include <TMIV/Common/Json.h>
#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/MivBitstream/Formatters.h>

#include <fstream>
#include <vector>

using namespace std::string_view_literals;

using TMIV::Common::logError;
using TMIV::Common::logInfo;

auto main(int argc, const char *argv[]) -> int32_t {
  try {
    const auto args = std::vector(argv, argv + argc);

    if ((args.size() != 5 && args.size() != 7) || args[1] != "-b"sv || args[3] != "-o"sv ||
        (args.size() == 7 && args[5] != "-nf"sv)) {
      logInfo("Usage: EgaInserter -b BITSTREAM -o HLS_LOG_FILE [-nf nframes]");
      return 1;
    }
    std::ifstream inStream{args[2], std::ios::binary};
    if (!inStream.good()) {
      logError("Failed to open {} for reading.\n", args[2]);
      return 1;
    }
    std::ofstream outStream{args[4], std::ios::binary};
    if (!outStream.good()) {
      logError("Failed to open {} for writing.\n", args[4]);
    }
    int32_t nframes = 0;
    if (args.size() == 7) {
      nframes = atoi(args[6]);
    }
    // Read the frame info to be inserted.
    std::vector<TMIV::Common::Json> seiJsons;
    for (int32_t frame = 0; frame < nframes; frame++) {
      std::string seiFile = "frame" + std::to_string(frame) + ".json";
      std::ifstream stream{seiFile};
      const auto json = TMIV::Common::Json::loadFrom(stream);
      seiJsons.emplace_back(json);
    }
    std::ofstream recodedStream{"recoded.bit", std::ios::binary};
    TMIV::EgaInserter::EgaInserter egaInserter{outStream, &recodedStream, seiJsons};
    egaInserter.parseV3cSampleStream(inStream);
    return 0;
  } catch (...) {
    return TMIV::Common::handleException();
  }
}
