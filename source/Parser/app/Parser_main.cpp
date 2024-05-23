/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ISO/IEC
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

#include <TMIV/Parser/Parser.h>

#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/Formatters.h>

#include <fstream>
#include <vector>

using namespace std::string_view_literals;

using TMIV::Common::logError;
using TMIV::Common::logInfo;

auto main(int argc, const char *argv[]) -> int32_t {
  try {
    const auto args = std::vector(argv, argv + argc);

    if (args.size() != 5 || args[1] != "-b"sv || args[3] != "-o"sv) {
      logInfo("Usage: TmivParser -b BITSTREAM -o HLS_LOG_FILE");
      return 1;
    }

    std::ifstream inStream{args[2], std::ios::binary};

    if (!inStream.good()) {
      logError("Failed to open {} for reading.", args[2]);
      return 1;
    }

    std::ofstream outStream{args[4], std::ios::binary};

    if (!outStream.good()) {
      logError("Failed to open {} for writing.", args[4]);
      return 1;
    }

    TMIV::Parser::Parser parser{outStream};
    parser.parseV3cSampleStream(inStream);
    return 0;
  } catch (...) {
    return TMIV::Common::handleException();
  }
}
