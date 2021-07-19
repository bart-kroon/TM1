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

#ifndef TMIV_MIVBITSTREAM_DECODEDATLASINFORMATIONHASH_H
#error "Include the .h, not the .hpp"
#endif

#include <TMIV/Common/verify.h>

namespace TMIV::MivBitstream {
inline auto DecodedAtlasHash::daih_atlas_md5(uint8_t i) const -> uint8_t {
  VERIFY_MIVBITSTREAM(m_daih_atlas_md5.has_value());
  VERIFY_MIVBITSTREAM(i < m_daih_atlas_md5.value().size());
  return m_daih_atlas_md5.value()[i];
}

inline auto DecodedAtlasHash::daih_atlas_crc() const -> uint16_t {
  VERIFY_MIVBITSTREAM(m_daih_atlas_crc.has_value());
  return *m_daih_atlas_crc;
}

inline auto DecodedAtlasHash::daih_atlas_checksum() const -> uint32_t {
  VERIFY_MIVBITSTREAM(m_daih_atlas_checksum.has_value());
  return *m_daih_atlas_checksum;
}

inline auto DecodedAtlasInformationHash::daih_cancel_flag() const -> bool {
  return m_daih_cancel_flag;
}

inline auto DecodedAtlasInformationHash::daih_persistence_flag() const -> bool {
  VERIFY_MIVBITSTREAM(m_daih_persistence_flag.has_value());
  return *m_daih_persistence_flag;
}

inline auto DecodedAtlasInformationHash::daih_hash_type() const -> uint8_t {
  VERIFY_MIVBITSTREAM(m_daih_hash_type.has_value());
  return *m_daih_hash_type;
}

inline auto DecodedAtlasInformationHash::daih_decoded_high_level_hash_present_flag() const -> bool {
  VERIFY_MIVBITSTREAM(m_daih_decoded_high_level_hash_present_flag.has_value());
  return *m_daih_decoded_high_level_hash_present_flag;
}

inline auto DecodedAtlasInformationHash::daih_decoded_atlas_hash_present_flag() const -> bool {
  VERIFY_MIVBITSTREAM(m_daih_decoded_atlas_hash_present_flag.has_value());
  return *m_daih_decoded_atlas_hash_present_flag;
}

inline auto DecodedAtlasInformationHash::daih_decoded_atlas_b2p_hash_present_flag() const -> bool {
  VERIFY_MIVBITSTREAM(m_daih_decoded_atlas_b2p_hash_present_flag.has_value());
  return *m_daih_decoded_atlas_b2p_hash_present_flag;
}

inline auto DecodedAtlasInformationHash::daih_decoded_atlas_tiles_hash_present_flag() const
    -> bool {
  VERIFY_MIVBITSTREAM(m_daih_decoded_atlas_tiles_hash_present_flag.has_value());
  return *m_daih_decoded_atlas_tiles_hash_present_flag;
}

inline auto DecodedAtlasInformationHash::daih_decoded_atlas_tiles_b2p_hash_present_flag() const
    -> bool {
  VERIFY_MIVBITSTREAM(m_daih_decoded_atlas_tiles_b2p_hash_present_flag.has_value());
  return *m_daih_decoded_atlas_tiles_b2p_hash_present_flag;
}

inline auto DecodedAtlasInformationHash::decoded_high_level_hash() const
    -> const DecodedHighLevelHash & {
  VERIFY_MIVBITSTREAM(m_decoded_high_level_hash.has_value());
  return *m_decoded_high_level_hash;
}

inline auto DecodedAtlasInformationHash::decoded_atlas_hash() const -> const DecodedAtlasHash & {
  VERIFY_MIVBITSTREAM(m_decoded_atlas_hash.has_value());
  return *m_decoded_atlas_hash;
}

inline auto DecodedAtlasInformationHash::decoded_atlas_b2p_hash() const
    -> const DecodedAtlasB2pHash & {
  VERIFY_MIVBITSTREAM(m_decoded_atlas_b2p_hash.has_value());
  return *m_decoded_atlas_b2p_hash;
}

inline auto DecodedAtlasInformationHash::daih_num_tiles_minus1() const -> uint8_t {
  VERIFY_MIVBITSTREAM(m_daih_num_tiles_minus1.has_value());
  return *m_daih_num_tiles_minus1;
}

inline auto DecodedAtlasInformationHash::daih_tile_id_len_minus1() const -> uint8_t {
  VERIFY_MIVBITSTREAM(m_daih_tile_id_len_minus1.has_value());
  return *m_daih_tile_id_len_minus1;
}

inline auto DecodedAtlasInformationHash::daih_tile_id(uint8_t tileIndex) const -> uint8_t {
  VERIFY_MIVBITSTREAM(m_daih_tile_id.has_value());
  VERIFY_MIVBITSTREAM(tileIndex < m_daih_tile_id.value().size());
  return m_daih_tile_id.value()[tileIndex];
}

inline auto DecodedAtlasInformationHash::decoded_atlas_tile_hash() const
    -> const DecodedAtlasTileHash & {
  VERIFY_MIVBITSTREAM(m_decoded_atlas_tile_hash.has_value());
  return *m_decoded_atlas_tile_hash;
}

inline auto DecodedAtlasInformationHash::decoded_atlas_tile_b2p_hash() const
    -> const DecodedAtlasTileB2pHash & {
  VERIFY_MIVBITSTREAM(m_decoded_atlas_tile_b2p_hash.has_value());
  return *m_decoded_atlas_tile_b2p_hash;
}

inline auto DecodedAtlasHash::daih_atlas_md5(uint8_t i, uint8_t value) -> auto & {
  if (!m_daih_atlas_md5.has_value()) {
    m_daih_atlas_md5.emplace(16);
  }
  VERIFY_MIVBITSTREAM(i < m_daih_atlas_md5.value().size());
  m_daih_atlas_md5.value()[i] = value;
  return *this;
}

inline auto DecodedAtlasHash::daih_atlas_crc(uint16_t value) -> auto & {
  m_daih_atlas_crc = value;
  return *this;
}

inline auto DecodedAtlasHash::daih_atlas_checksum(uint32_t value) -> auto & {
  m_daih_atlas_checksum = value;
  return *this;
}

inline auto DecodedAtlasInformationHash::daih_cancel_flag(bool value) -> auto & {
  m_daih_cancel_flag = value;
  return *this;
}

inline auto DecodedAtlasInformationHash::daih_persistence_flag(bool value) -> auto & {
  m_daih_persistence_flag = value;
  return *this;
}

inline auto DecodedAtlasInformationHash::daih_hash_type(uint8_t value) -> auto & {
  m_daih_hash_type = value;
  return *this;
}

inline auto DecodedAtlasInformationHash::daih_decoded_high_level_hash_present_flag(bool value)
    -> auto & {
  m_daih_decoded_high_level_hash_present_flag = value;
  return *this;
}

inline auto DecodedAtlasInformationHash::daih_decoded_atlas_hash_present_flag(bool value)
    -> auto & {
  m_daih_decoded_atlas_hash_present_flag = value;
  return *this;
}

inline auto DecodedAtlasInformationHash::daih_decoded_atlas_b2p_hash_present_flag(bool value)
    -> auto & {
  m_daih_decoded_atlas_b2p_hash_present_flag = value;
  return *this;
}

inline auto DecodedAtlasInformationHash::daih_decoded_atlas_tiles_hash_present_flag(bool value)
    -> auto & {
  m_daih_decoded_atlas_tiles_hash_present_flag = value;
  return *this;
}

inline auto DecodedAtlasInformationHash::daih_decoded_atlas_tiles_b2p_hash_present_flag(bool value)
    -> auto & {
  m_daih_decoded_atlas_tiles_b2p_hash_present_flag = value;
  return *this;
}

inline auto DecodedAtlasInformationHash::decoded_high_level_hash(DecodedHighLevelHash &dhlh)
    -> auto & {
  m_decoded_high_level_hash = dhlh;
  return *this;
}

inline auto DecodedAtlasInformationHash::decoded_atlas_hash(DecodedAtlasHash &dah) -> auto & {
  m_decoded_atlas_hash = dah;
  return *this;
}

inline auto DecodedAtlasInformationHash::decoded_atlas_b2p_hash(DecodedAtlasB2pHash &dabh)
    -> auto & {
  m_decoded_atlas_b2p_hash = dabh;
  return *this;
}

inline auto DecodedAtlasInformationHash::daih_num_tiles_minus1(uint8_t value) -> auto & {
  m_daih_num_tiles_minus1 = value;
  return *this;
}

inline auto DecodedAtlasInformationHash::daih_tile_id_len_minus1(uint8_t value) -> auto & {
  m_daih_tile_id_len_minus1 = value;
  return *this;
}

inline auto DecodedAtlasInformationHash::daih_tile_id(uint8_t tileIndex, uint8_t value) -> auto & {
  if (!m_daih_tile_id.has_value()) {
    m_daih_tile_id.emplace(*m_daih_num_tiles_minus1 + 1);
  }
  VERIFY_MIVBITSTREAM(tileIndex < m_daih_tile_id.value().size());
  m_daih_tile_id.value()[tileIndex] = value;
  return *this;
}

inline auto DecodedAtlasInformationHash::decoded_atlas_tile_hash(DecodedAtlasTileHash &dath)
    -> auto & {
  m_decoded_atlas_tile_hash = dath;
  return *this;
}

inline auto DecodedAtlasInformationHash::decoded_atlas_tile_b2p_hash(DecodedAtlasTileB2pHash &datbh)
    -> auto & {
  m_decoded_atlas_tile_b2p_hash = datbh;
  return *this;
}
} // namespace TMIV::MivBitstream
