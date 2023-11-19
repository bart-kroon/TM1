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

#ifndef TMIV_MIVBITSTREAM_FORMATTERS_H
#define TMIV_MIVBITSTREAM_FORMATTERS_H

#include <TMIV/Common/Formatters.h>

#include "AtlasId.h"
#include "AtlasTileLayerRBSP.h"
#include "NalUnit.h"
#include "V3cParameterSet.h"
#include "V3cUnit.h"
#include "ViewId.h"

namespace TMIV_FMT {
namespace miv = ::TMIV::MivBitstream;

template <> struct formatter<miv::AiAttributeTypeId> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::AthType> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::AtlasId> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::FlexiblePatchOrientation> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::NalUnit> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::NalUnitHeader> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::NalUnitType> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::PtlLevelIdc> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::PtlMaxDecodesIdc> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::PtlProfileCodecGroupIdc> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::PtlProfileReconstructionIdc> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::PtlProfileToolsetIdc> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::V3cUnit> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::V3cUnitHeader> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::V3cUnitPayload> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::ViewId> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::VpsExtensionType> : TMIV::Common::OstreamFormatter {};
template <> struct formatter<miv::VuhUnitType> : TMIV::Common::OstreamFormatter {};
} // namespace TMIV_FMT

#endif
