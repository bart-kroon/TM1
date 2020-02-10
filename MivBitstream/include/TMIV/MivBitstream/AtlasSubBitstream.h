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

#ifndef _TMIV_MIVBITSTREAM_ATLASSUBBITSTREAM_H_
#define _TMIV_MIVBITSTREAM_ATLASSUBBITSTREAM_H_

#include <TMIV/MivBitstream/AtlasFrameParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>
#include <TMIV/MivBitstream/NalSampleStream.h>

#include <optional>

namespace TMIV::MivBitstream {
// 23090-5: atlas_sub_bitstream()
class AtlasSubBitstream {
public:
  AtlasSubBitstream() = default;
  explicit AtlasSubBitstream(const NalSampleStream &nss) : m_nss{nss} {}
  explicit AtlasSubBitstream(const SampleStreamNalHeader &ssnh)
      : AtlasSubBitstream{NalSampleStream{ssnh}} {}
  AtlasSubBitstream(const AtlasSubBitstream &) = default;
  AtlasSubBitstream(AtlasSubBitstream &&) = default;
  AtlasSubBitstream &operator=(const AtlasSubBitstream &) = default;
  AtlasSubBitstream &operator=(AtlasSubBitstream &&) = default;
  virtual ~AtlasSubBitstream() = default;

  const auto &nal_sample_stream() const noexcept;
  const auto &atlas_sequence_parameter_sets() const noexcept { return m_asps; }
  const auto &atlas_frame_parameter_sets() const noexcept { return m_afps; }

  friend auto operator<<(std::ostream &stream, const AtlasSubBitstream &x) -> std::ostream &;

  auto operator==(const AtlasSubBitstream &other) const noexcept -> bool;
  auto operator!=(const AtlasSubBitstream &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream) -> AtlasSubBitstream;
  void encodeTo(std::ostream &stream) const;

protected:
  virtual void decodeNalUnit(const NalUnit &nal_unit);

private:
  void decodeAsps(const NalUnit &nal_unit);
  void decodeAfps(const NalUnit &nal_unit);

  std::optional<NalSampleStream> m_nss;
  std::vector<AtlasSequenceParameterSetRBSP> m_asps;
  std::vector<AtlasFrameParameterSetRBSP> m_afps;
};
} // namespace TMIV::MivBitstream

#endif
