/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ITU/ISO/IEC
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

#include <TMIV/Common/Common.h>

#include "../PartitionImpl.h"

#include <TLibDecoder/NALread.h>

#include <cstring>

using namespace std::string_literals;

namespace TMIV::VideoDecoder {
namespace {
auto nalUnitType(const std::string &unit) -> NalUnitType {
  InputNALUnit nalu;
  nalu.getBitstream().getFifo().resize(unit.size());
  memcpy(nalu.getBitstream().getFifo().data(), unit.data(), unit.size());
  read(nalu);
  return nalu.m_nalUnitType;
}

auto isIrapStart(const std::string &unit) -> bool {
  const auto nut = nalUnitType(unit);

  return (NAL_UNIT_CODED_SLICE_BLA_W_LP <= nut && nut <= NAL_UNIT_RESERVED_IRAP_VCL23) ||
         nut == NAL_UNIT_VPS || nut == NAL_UNIT_SPS;
}

auto isIrapVcf(const std::string &unit) -> bool {
  const auto nut = nalUnitType(unit);

  return NAL_UNIT_CODED_SLICE_BLA_W_LP <= nut && nut <= NAL_UNIT_RESERVED_IRAP_VCL23;
}
} // namespace

auto partitionHevcMain10(NalUnitSource source) -> CodedVideoSequenceSource {
  return partition(std::move(source), isIrapStart, isIrapVcf);
}

auto partitionHevc444(NalUnitSource source) -> CodedVideoSequenceSource {
  return partitionHevcMain10(std::move(source));
}
} // namespace TMIV::VideoDecoder
