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
#define _TMIV_IO_IO_H_

#include <TMIV/Common/Frame.h>
#include <TMIV/Common/Json.h>
#include <TMIV/MivBitstream/AccessUnit.h>
#include <TMIV/MivBitstream/IvAccessUnitParams.h>
#include <TMIV/MivBitstream/IvSequenceParams.h>

// Functions for file I/O
//
// Frame indices are zero-based and relative to the StartFrame parameter.
// These functions will print something short to screen.
namespace TMIV::IO {
// Load sequence metadata from the configuration files. It is up to the Encoder to comply (or
// ignore) fields such as num_groups. The in-memory metadata representation has to be complete
// only after IEncoder.
auto loadSourceIvSequenceParams(const Common::Json &config) -> MivBitstream::IvSequenceParams;

// Loads a source frame including entity maps when applicable
auto loadSourceFrame(const Common::Json &config, const Common::SizeVector &sizes, int frameIndex)
    -> Common::MVD16Frame;

void saveAtlas(const Common::Json &config, int frameIndex, const Common::MVD10Frame &frame);

void saveBlockToPatchMaps(const Common::Json &config, int frameIndex,
                          const MivBitstream::AccessUnit &frame);
void savePrunedFrame(const Common::Json &config, int frameIndex,
                     const std::pair<std::vector<Common::Texture444Depth10Frame>, Common::MaskList>
                         &prunedViewsAndMasks);

auto loadViewportMetadata(const Common::Json &config, int frameIndex) -> MivBitstream::ViewParams;
void saveViewport(const Common::Json &config, int frameIndex,
                  const Common::TextureDepth16Frame &frame);
} // namespace TMIV::IO

#include "IO.hpp"

#endif
