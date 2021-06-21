/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#ifndef TMIV_ENCODER_GROUPBASEDENCODER_H
#define TMIV_ENCODER_GROUPBASEDENCODER_H

#include <TMIV/Encoder/Encoder.h>

namespace TMIV::Encoder {
class GroupBasedEncoder {
public:
  // Make an Encoder per group
  GroupBasedEncoder(const Common::Json &rootNode, const Common::Json &componentNode);

  GroupBasedEncoder(const GroupBasedEncoder &) = delete;
  GroupBasedEncoder(GroupBasedEncoder &&) = default;
  auto operator=(const GroupBasedEncoder &) -> GroupBasedEncoder & = delete;
  auto operator=(GroupBasedEncoder &&) -> GroupBasedEncoder & = default;
  ~GroupBasedEncoder() = default;

  // Let each per-group encoder prepare the sequence
  void prepareSequence(const MivBitstream::SequenceConfig &sequenceConfig,
                       const Common::MVD16Frame &firstFrame);

  // Let each per-group encoder prepare the access unit
  void prepareAccessUnit();

  // Push frame to each per-group encoder
  void pushFrame(const Common::MVD16Frame &views);

  // Let each per-group encoer complete the access unit and merge the metadata
  auto completeAccessUnit() -> const EncoderParams &;

  // Pop atlases from each group and merge them into a single array
  auto popAtlas() -> Common::MVD10Frame;

  // Maximum aggregated luma samples per frame all groups combined
  [[nodiscard]] auto maxLumaSamplesPerFrame() const -> size_t;

private:
  // A grouping as an array of groupId-viewId pairs
  using Grouping = std::vector<std::pair<size_t, size_t>>;

  // Partition the views, thereby forming the groups
  virtual auto sourceSplitter(const MivBitstream::SequenceConfig &sequenceConfig) -> Grouping;

  // Split per-group sequence configuration
  [[nodiscard]] auto
  filterSourceCameraNames(size_t groupId, const MivBitstream::SequenceConfig &sequenceConfig) const
      -> MivBitstream::SequenceConfig;

  // Split per-group views
  [[nodiscard]] auto splitViews(size_t groupId, const Common::MVD16Frame &views) const
      -> Common::MVD16Frame;

  static auto mergeVps(const std::vector<const MivBitstream::V3cParameterSet *> &vps)
      -> MivBitstream::V3cParameterSet;

  // Merge per-group encoder parameters
  auto mergeParams(const std::vector<const EncoderParams *> & /*perGroupParams*/)
      -> const EncoderParams &;

  uint8_t m_numGroups;
  Grouping m_grouping;
  std::vector<Encoder> m_encoders;
  EncoderParams m_params;
  std::vector<const EncoderParams *> m_perGroupParams;
};
} // namespace TMIV::Encoder

#endif
