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

#ifndef _TMIV_ENCODER_GROUPBASEDENCODER_H_
#define _TMIV_ENCODER_GROUPBASEDENCODER_H_

#include <TMIV/Encoder/Encoder.h>

namespace TMIV::Encoder {
class GroupBasedEncoder : public IEncoder {
public:
  // Make an Encoder per group
  GroupBasedEncoder(const Common::Json &rootNode, const Common::Json &componentNode);

  GroupBasedEncoder(const GroupBasedEncoder &) = delete;
  GroupBasedEncoder(GroupBasedEncoder &&) = default;
  auto operator=(const GroupBasedEncoder &) -> GroupBasedEncoder & = delete;
  auto operator=(GroupBasedEncoder &&) -> GroupBasedEncoder & = default;
  ~GroupBasedEncoder() override = default;

  // Let each per-group encoder prepare the sequence and merge the metadata
  auto prepareSequence(MivBitstream::IvSequenceParams ivSequenceParams)
      -> const MivBitstream::IvSequenceParams & override;

  // Let each per-group encoder prepare the access unit
  void prepareAccessUnit(MivBitstream::IvAccessUnitParams ivAccessUnitParams) override;

  // Push frame to each per-group encoder
  void pushFrame(Common::MVD16Frame views) override;

  // Let each per-group encoer complete the access unit and merge the metadata
  auto completeAccessUnit() -> const MivBitstream::IvAccessUnitParams & override;

  // Pop atlases from each group and merge them into a single array
  auto popAtlas() -> Common::MVD10Frame override;

  // Maximum aggregated luma samples per frame all groups combined
  auto maxLumaSamplesPerFrame() const -> std::size_t override;

protected:
  // A grouping as an array of groupId-viewId pairs
  using Grouping = std::vector<std::pair<std::size_t, std::size_t>>;

  // Partition the views, thereby forming the groups
  virtual auto sourceSplitter(const MivBitstream::IvSequenceParams &ivSequenceParams) -> Grouping;

protected:
  // Split per-group sequence parameters
  virtual auto splitSequenceParams(size_t groupId,
                                   const MivBitstream::IvSequenceParams &ivSequenceParams) const
      -> MivBitstream::IvSequenceParams;

  // Split per-group views
  virtual auto splitViews(size_t groupId, Common::MVD16Frame &views) const -> Common::MVD16Frame;

  // Merge per-group sequence parameters
  virtual auto mergeSequenceParams(const std::vector<const MivBitstream::IvSequenceParams *> &)
      -> const MivBitstream::IvSequenceParams &;

  // Merge per-group access unit parameters
  virtual auto mergeAccessUnitParams(const std::vector<const MivBitstream::IvAccessUnitParams *> &)
      -> const MivBitstream::IvAccessUnitParams &;

private:
  auto numGroups() const -> std::size_t { return m_encoders.size(); }

  Grouping m_grouping;
  std::vector<Encoder> m_encoders;
  MivBitstream::IvSequenceParams m_ivSequenceParams;
  std::vector<const MivBitstream::IvSequenceParams *> m_perGroupSequenceParams;
  MivBitstream::IvAccessUnitParams m_ivAccessUnitParams;
};
} // namespace TMIV::Encoder

#endif
