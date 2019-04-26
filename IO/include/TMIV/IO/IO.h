/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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
#include <TMIV/Metadata/CameraParameterList.h>
#include <TMIV/Metadata/PatchParameterList.h>
#include <TMIV/ViewOptimizer/IViewOptimizer.h>

// Functions for file I/O
//
// Frame indices are zero-based and relative to the StartFrame parameter.
// These functions will print something short to screen.
namespace TMIV::IO {
template <class T>
using BaseAdditional = ViewOptimizer::IViewOptimizer::Output<T>;

Metadata::CameraParameterList loadSourceMetadata(const Common::Json &config);
Common::MVD16Frame loadSourceFrame(const Common::Json &config,
                                   const Metadata::CameraParameterList &cameras,
                                   int frameIndex);

void saveOptimizedFrame(
    const Common::Json &config, int frameIndex,
    const BaseAdditional<Metadata::CameraParameterList> &cameras,
    const BaseAdditional<Common::MVD16Frame> &frame);
auto loadOptimizedFrame(
    const Common::Json &config,
    const BaseAdditional<Metadata::CameraParameterList> &cameras,
    int frameIndex) -> BaseAdditional<Common::MVD16Frame>;
void saveOptimizedMetadata(
    const Common::Json &config, int frameIndex,
    const BaseAdditional<Metadata::CameraParameterList> &metadata);
auto loadOptimizedMetadata(const Common::Json &config, int frameIndex)
    -> BaseAdditional<Metadata::CameraParameterList>;

void saveTransportFrame(const Common::Json &config, int frameIndex,
                        const Common::MVD16Frame &frame);

struct MivMetadata {
  std::vector<Common::Vec2i> atlasSize;
  Metadata::PatchParameterList patches;
  Metadata::CameraParameterList cameras;
};

void saveMivMetadata(const Common::Json &config, int frameIndex,
                     const MivMetadata &metadata);
auto loadMivMetadata(const Common::Json &config, int frameIndex) -> MivMetadata;

void savePatchList(const Common::Json &config, const std::string &name,
                   Metadata::PatchParameterList patches);

// Save the atlas (10-bit 4:2:0 texture, 16-bit depth) with depth converted to
// 10-bit
void saveAtlas(const Common::Json &config, int frameIndex,
               Common::MVD16Frame const &frame);
auto loadAtlas(const Common::Json &config,
               const std::vector<Common::Vec2i> &atlasSize, int frameIndex)
    -> Common::MVD10Frame;

void savePatchIdMaps(const Common::Json &config, int frameIndex,
                     const Common::PatchIdMapList &maps);
auto loadPatchIdMaps(const Common::Json &config,
                     const std::vector<Common::Vec2i> &atlasSize,
                     int frameIndex) -> Common::PatchIdMapList;

auto loadViewportMetadata(const Common::Json &config, int frameIndex)
    -> Metadata::CameraParameters;
void saveViewport(const Common::Json &config, int frameIndex,
                  const Common::TextureDepth10Frame &frame);
} // namespace TMIV::IO

#endif
