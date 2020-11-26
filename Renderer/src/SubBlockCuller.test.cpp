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

#include <catch2/catch.hpp>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/Vector.h>
#include <TMIV/MivBitstream/PatchParamsList.h>
#include <TMIV/MivBitstream/ViewParamsList.h>
#include <TMIV/Renderer/SubBlockCuller.h>
#include <TMIV/Renderer/reprojectPoints.h>


#include <fstream>
#include <iostream>
#include <string>
#include <regex>
using namespace std;

using namespace TMIV::Common;
using namespace TMIV::MivBitstream;
using namespace TMIV::Renderer;
//using namespace TMIV::IO;

namespace {

  struct Pose {
    Vec3f position;
    Vec3f rotation;
  };

  auto loadPoseFromCSV(istream &stream, int frameIndex) -> Pose {
    string line;
    getline(stream, line);

    regex re_header(R"(\s*X\s*,\s*Y\s*,\s*Z\s*,\s*Yaw\s*,\s*Pitch\s*,\s*Roll\s*)");
    if (!regex_match(line, re_header)) {
      throw runtime_error("Format error in the pose trace header");
    }

    int currentFrameIndex = 0;
    regex re_row("([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+)");
    regex re_empty("\\s*");
    bool trailing_empty_lines = false;

    while (getline(stream, line)) {
      smatch match;
      if (!trailing_empty_lines && regex_match(line, match, re_row)) {
        if (currentFrameIndex == frameIndex) {
          return { Vec3f({stof(match[1].str()), stof(match[2].str()), stof(match[3].str())}),
                  Vec3f({stof(match[4].str()), stof(match[5].str()), stof(match[6].str())}) };
        }
        { currentFrameIndex++; }
      }
      else if (regex_match(line, re_empty)) {
        trailing_empty_lines = true;
      }
      else {
        throw runtime_error("Format error in a pose trace row");
      }
    }

    throw runtime_error("Unable to load required frame index " + to_string(frameIndex));
  }
} // namespace
