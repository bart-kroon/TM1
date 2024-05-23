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

#ifndef TMIV_PTLCHECKER_ABSTRACTCHECKER_H
#define TMIV_PTLCHECKER_ABSTRACTCHECKER_H

#include <TMIV/MivBitstream/AccessUnit.h>
#include <TMIV/MivBitstream/CommonAtlasFrameRBSP.h>

namespace TMIV::PtlChecker {
class AbstractChecker {
public:
  AbstractChecker() = default;
  AbstractChecker(const AbstractChecker &) = delete;
  AbstractChecker(AbstractChecker &&) = default;
  auto operator=(const AbstractChecker &) -> AbstractChecker & = delete;
  auto operator=(AbstractChecker &&) -> AbstractChecker & = default;
  virtual ~AbstractChecker() = default;

  using Logger = std::function<void(const std::string &failure)>;

  // To support experimentation, the PTL checker only logs a warning. This behaviour can be changed
  // by replacing the log function.
  virtual void replaceLogger(Logger value) = 0;

  virtual void checkVuh(const MivBitstream::V3cUnitHeader &vuh) = 0;
  virtual void checkNuh(const MivBitstream::NalUnitHeader &nuh) = 0;
  virtual void checkAndActivateVps(const MivBitstream::V3cParameterSet &vps) = 0;
  virtual void
  checkAndActivateCasps(const MivBitstream::CommonAtlasSequenceParameterSetRBSP &casps) = 0;
  virtual void checkAsps(MivBitstream::AtlasId atlasId,
                         const MivBitstream::AtlasSequenceParameterSetRBSP &asps) = 0;
  virtual void checkAfps(const MivBitstream::AtlasFrameParameterSetRBSP &afps) = 0;
  virtual void checkAtl(const MivBitstream::NalUnitHeader &nuh,
                        const MivBitstream::AtlasTileLayerRBSP &atl) = 0;
  virtual void checkCaf(const MivBitstream::NalUnitHeader &nuh,
                        const MivBitstream::CommonAtlasFrameRBSP &caf) = 0;
  virtual void checkVideoFrame(MivBitstream::VuhUnitType vut,
                               const MivBitstream::AtlasSequenceParameterSetRBSP &asps,
                               const Common::Frame<> &frame) = 0;
  virtual void checkV3cFrame(const MivBitstream::AccessUnit &frame) = 0;
};

using SharedChecker = std::shared_ptr<PtlChecker::AbstractChecker>;
} // namespace TMIV::PtlChecker

#endif
