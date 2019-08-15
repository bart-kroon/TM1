/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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
#include <TMIV/Metadata/AtlasParametersList.h>
#include <TMIV/Metadata/CameraParametersList.h>
#include <TMIV/ViewOptimizer/IViewOptimizer.h>

// Functions for file I/O
//
// Frame indices are zero-based and relative to the StartFrame parameter.
// These functions will print something short to screen.
namespace TMIV::IO {
template <class T> using BasicAdditional = ViewOptimizer::IViewOptimizer::Output<T>;

auto sizesOf(const Metadata::CameraParametersList &cameras) -> std::vector<Common::Vec2i>;
Metadata::CameraParametersList loadSourceMetadata(const Common::Json &config);
Common::MVD16Frame loadSourceFrame(const Common::Json &config,
                                   const std::vector<Common::Vec2i> &sizes, int frameIndex);

void saveOptimizedFrame(const Common::Json &config, int frameIndex,
                        const BasicAdditional<Common::MVD16Frame> &frame);
auto loadOptimizedFrame(const Common::Json &config,
                        const BasicAdditional<std::vector<Common::Vec2i>> &sizes, int frameIndex)
    -> BasicAdditional<Common::MVD16Frame>;
void saveOptimizedMetadata(const Common::Json &config, int frameIndex,
                           const BasicAdditional<Metadata::CameraParametersList> &metadata);
auto loadOptimizedMetadata(const Common::Json &config, int frameIndex)
    -> BasicAdditional<Metadata::CameraParametersList>;

void savePrunedFrame(const Common::Json &config, int frameIndex, const Common::MVD16Frame &frame);

struct MivMetadata {
  std::vector<Common::Vec2i> atlasSize; // atlas_width/height in MPEG/N18576
  bool omafV1CompatibleFlag{};          // omaf_v1_compatible in MPEG/N18576
  Metadata::AtlasParametersList patches;
  Metadata::CameraParametersList cameras;

  bool operator==(const MivMetadata &other) const;
};

void saveMivMetadata(const Common::Json &config, int frameIndex, const MivMetadata &metadata);
auto loadMivMetadata(const Common::Json &config, int frameIndex) -> MivMetadata;

// Save the atlas (10-bit 4:2:0 texture, 16-bit depth) with depth converted to
// 10-bit
void saveAtlas(const Common::Json &config, int frameIndex, const Common::MVD16Frame &frame);
void saveAtlas(const Common::Json &config, int frameIndex, const Common::MVD10Frame &frame);
auto loadAtlas(const Common::Json &config, const std::vector<Common::Vec2i> &atlasSize,
               int frameIndex) -> Common::MVD10Frame;
auto loadAtlasAndDecompress(const Common::Json &config, const std::vector<Common::Vec2i> &atlasSize,
                            int frameIndex) -> Common::MVD16Frame;

void savePatchIdMaps(const Common::Json &config, int frameIndex,
                     const Common::PatchIdMapList &maps);
auto loadPatchIdMaps(const Common::Json &config, const std::vector<Common::Vec2i> &atlasSize,
                     int frameIndex) -> Common::PatchIdMapList;

auto loadViewportMetadata(const Common::Json &config, int frameIndex) -> Metadata::CameraParameters;
void saveViewport(const Common::Json &config, int frameIndex,
                  const Common::TextureDepth16Frame &frame);

// Returns a pair of metadata and frame indices to pass to loadMivMetadata and
// loadAtlas. If frameIndex is strictly less than the actual number of frames in
// the encoded stream, then regular values are returned else mirrored indices
// are computed.
std::pair<int, int> getExtendedIndex(const Common::Json &config, int frameIndex);

} // namespace TMIV::IO

#endif
