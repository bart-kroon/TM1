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

#ifndef _TMIV_ATLASCONSTRUCTOR_AGGREGATOR_H_
#define _TMIV_ATLASCONSTRUCTOR_AGGREGATOR_H_

#include <TMIV/AtlasConstructor/IAggregator.h>
#include <TMIV/Common/Json.h>

namespace TMIV::AtlasConstructor {

// The Aggregator of TMIV 1.0 provided by Technicolor
class Aggregator : public IAggregator {
public:
  Aggregator(const Common::Json & /*unused*/, const Common::Json & /*unused*/);
  Aggregator(const Aggregator &) = delete;
  Aggregator(Aggregator &&) = default;
  Aggregator &operator=(const Aggregator &) = delete;
  Aggregator &operator=(Aggregator &&) = default;
  ~Aggregator() override = default;

  void prepareIntraPeriod() override;
  void pushMask(const MaskList &mask) override;
  void completeIntraPeriod() override {}
  const MaskList &getAggregatedMask() const override {
    return m_aggregatedMask;
  }

private:
  MaskList m_aggregatedMask;
};

} // namespace TMIV::AtlasConstructor

#endif
