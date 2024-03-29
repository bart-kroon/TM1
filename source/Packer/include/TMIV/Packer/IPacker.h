/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#ifndef TMIV_PACKER_IPACKER_H
#define TMIV_PACKER_IPACKER_H

#include <TMIV/Common/Frame.h>
#include <TMIV/MivBitstream/PatchParamsList.h>
#include <TMIV/MivBitstream/ViewParamsList.h>

namespace TMIV::Packer {
class IPacker {
public:
  IPacker() = default;
  IPacker(const IPacker &) = delete;
  IPacker(IPacker &&) = default;
  auto operator=(const IPacker &) -> IPacker & = delete;
  auto operator=(IPacker &&) -> IPacker & = default;
  virtual ~IPacker() = default;

  virtual void initialize(const std::vector<Common::SizeVector> &atlasSizes, int32_t blockSize) = 0;
  virtual void initialize(std::vector<std::vector<MivBitstream::TilePartition>> tileSizes) = 0;
  virtual auto pack(const std::vector<Common::SizeVector> &atlasSize,
                    const Common::FrameList<uint8_t> &masks,
                    const MivBitstream::ViewParamsList &viewParamsList, int32_t blockSize,
                    const Common::FrameList<uint32_t> &information)
      -> MivBitstream::PatchParamsList = 0;
  virtual void
  updateAggregatedEntityMasks(const std::vector<Common::FrameList<uint8_t>> &entityMasks) = 0;
};
} // namespace TMIV::Packer

#endif
