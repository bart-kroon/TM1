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

#include <TMIV/MivBitstream/MivDecoder.h>
#include <TMIV/MivBitstream/MivDecoderMode.h>

#include <cstring>
#include <fstream>
#include <iostream>

namespace TMIV::MivBitstream {
const MivDecoderMode mode = MivDecoderMode::MIV;
}

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

auto main(int argc, char *argv[]) -> int {
  try {
    const auto args = vector(argv, argv + argc);

    if (args.size() != 3 || strcmp(args[1], "-b") != 0) {
      clog << "Usage: Parser -b BITSTREAM" << endl;
      return 1;
    }

    ifstream stream{args[2], ios::binary};
    if (!stream.good()) {
      clog << "Failed to open bitstream for reading" << endl;
      return 1;
    }

    auto decoder = MivDecoder{stream,
                              [](uint8_t /* atlasId */, uint32_t /* frameId */, Vec2i frameSize) {
                                return Depth10Frame{frameSize.x(), frameSize.y()};
                              },
                              [](uint8_t /* atlasId */, uint32_t /* frameId */, Vec2i frameSize) {
                                return Texture444Frame{frameSize.x(), frameSize.y()};
                              }};
    decoder.enableBitrateReporting();
    decoder.decode();
    ofstream bitrateReport{string(argv[2]) + ".csv"};
    decoder.printBitrateReport(bitrateReport);
    return 0;
  } catch (runtime_error &e) {
    clog << e.what() << endl;
    return 1;
  }
}
