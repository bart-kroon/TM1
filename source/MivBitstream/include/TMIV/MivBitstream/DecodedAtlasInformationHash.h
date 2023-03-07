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

#ifndef TMIV_MIVBITSTREAM_DECODEDATLASINFORMATIONHASH_H
#define TMIV_MIVBITSTREAM_DECODEDATLASINFORMATIONHASH_H

#include <TMIV/Common/Bitstream.h>

#include <optional>
#include <variant>
#include <vector>

namespace TMIV::MivBitstream {
// 23090-5: decoded_high_level_hash
class DecodedHighLevelHash {
public:
  friend auto operator<<(std::ostream &stream, const DecodedHighLevelHash & /*x*/)
      -> std::ostream & {
    return stream;
  };

  auto operator==(const DecodedHighLevelHash & /*other*/) const noexcept -> bool { return true; };

  static auto decodeFrom(Common::InputBitstream & /*bitstream*/, uint8_t /*hashType*/)
      -> DecodedHighLevelHash {
    return DecodedHighLevelHash{};
  };
  void encodeTo(Common::OutputBitstream & /*bitstream*/, uint8_t /*hashType*/) const {};
};

// 23090-5: decoded_atlas_hash
class DecodedAtlasHash {
public:
  [[nodiscard]] auto daih_atlas_md5(uint8_t i) const -> uint8_t;
  [[nodiscard]] auto daih_atlas_crc() const -> uint16_t;
  [[nodiscard]] auto daih_atlas_checksum() const -> uint32_t;

  auto daih_atlas_md5(uint8_t i, uint8_t value) -> auto &;
  auto daih_atlas_crc(uint16_t value) -> auto &;
  auto daih_atlas_checksum(uint32_t value) -> auto &;

  friend auto operator<<(std::ostream &stream, const DecodedAtlasHash &x) -> std::ostream &;

  auto operator==(const DecodedAtlasHash &other) const -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, uint8_t hashType) -> DecodedAtlasHash;
  void encodeTo(Common::OutputBitstream &bitstream, uint8_t hashType) const;

private:
  std::optional<std::vector<uint8_t>> m_daih_atlas_md5{};
  std::optional<uint16_t> m_daih_atlas_crc{};
  std::optional<uint32_t> m_daih_atlas_checksum{};
};

// 23090-5: decoded_atlas_b2p_hash
class DecodedAtlasB2pHash {
public:
  friend auto operator<<(std::ostream &stream, const DecodedAtlasB2pHash & /*x*/)
      -> std::ostream & {
    return stream;
  };

  auto operator==(const DecodedAtlasB2pHash & /*other*/) const noexcept -> bool { return true; };

  static auto decodeFrom(Common::InputBitstream & /*bitstream*/, uint8_t /*hashType*/)
      -> DecodedAtlasB2pHash {
    return DecodedAtlasB2pHash{};
  };
  void encodeTo(Common::OutputBitstream & /*bitstream*/, uint8_t /*hashType*/) const {};
};

// 23090-5: decoded_atlas_tile_hash
class DecodedAtlasTileHash {
public:
  friend auto operator<<(std::ostream &stream, const DecodedAtlasTileHash & /*x*/)
      -> std::ostream & {
    return stream;
  };

  auto operator==(const DecodedAtlasTileHash & /*other*/) const noexcept -> bool { return true; };

  static auto decodeFrom(Common::InputBitstream & /*bitstream*/, uint8_t /*hashType*/,
                         uint8_t /*tileId*/) -> DecodedAtlasTileHash {
    return DecodedAtlasTileHash{};
  };
  void encodeTo(Common::OutputBitstream & /*bitstream*/, uint8_t /*hashType*/,
                uint8_t /*tileId*/) const {};
};

// 23090-5: decoded_atlas_tile_b2p_hash
class DecodedAtlasTileB2pHash {
public:
  friend auto operator<<(std::ostream &stream, const DecodedAtlasTileB2pHash & /*x*/)
      -> std::ostream & {
    return stream;
  };

  auto operator==(const DecodedAtlasTileB2pHash & /*other*/) const noexcept -> bool {
    return true;
  };

  static auto decodeFrom(Common::InputBitstream & /*bitstream*/, uint8_t /*hashType*/,
                         uint8_t /*tileId*/) -> DecodedAtlasTileB2pHash {
    return DecodedAtlasTileB2pHash{};
  };
  void encodeTo(Common::OutputBitstream & /*bitstream*/, uint8_t /*hashType*/,
                uint8_t /*tileId*/) const {};
};

// 23090-5: decoded_atlas_information_hash ( payloadSize )
//
// Limitations of this implementation:
//   * daih_decoded_high_level_hash_present_flag == 0
//   * daih_decoded_atlas_b2p_hash_present_flag == 0
//   * daih_decoded_atlas_tiles_hash_present_flag == 0
//   * daih_decoded_atlas_tiles_b2p_hash_present_flag == 0
class DecodedAtlasInformationHash {
public:
  [[nodiscard]] auto daih_cancel_flag() const -> bool;
  [[nodiscard]] auto daih_persistence_flag() const -> bool;
  [[nodiscard]] auto daih_hash_type() const -> uint8_t;
  [[nodiscard]] auto daih_decoded_high_level_hash_present_flag() const -> bool;
  [[nodiscard]] auto daih_decoded_atlas_hash_present_flag() const -> bool;
  [[nodiscard]] auto daih_decoded_atlas_b2p_hash_present_flag() const -> bool;
  [[nodiscard]] auto daih_decoded_atlas_tiles_hash_present_flag() const -> bool;
  [[nodiscard]] auto daih_decoded_atlas_tiles_b2p_hash_present_flag() const -> bool;
  [[nodiscard]] auto decoded_high_level_hash() const -> const DecodedHighLevelHash &;
  [[nodiscard]] auto decoded_atlas_hash() const -> const DecodedAtlasHash &;
  [[nodiscard]] auto decoded_atlas_b2p_hash() const -> const DecodedAtlasB2pHash &;
  [[nodiscard]] auto daih_num_tiles_minus1() const -> uint8_t;
  [[nodiscard]] auto daih_tile_id_len_minus1() const -> uint8_t;
  [[nodiscard]] auto daih_tile_id(uint8_t tileId) const -> uint8_t;
  [[nodiscard]] auto decoded_atlas_tile_hash() const -> const DecodedAtlasTileHash &;
  [[nodiscard]] auto decoded_atlas_tile_b2p_hash() const -> const DecodedAtlasTileB2pHash &;

  auto daih_cancel_flag(bool value) -> auto &;
  auto daih_persistence_flag(bool value) -> auto &;
  auto daih_hash_type(uint8_t value) -> auto &;
  auto daih_decoded_high_level_hash_present_flag(bool value) -> auto &;
  auto daih_decoded_atlas_hash_present_flag(bool value) -> auto &;
  auto daih_decoded_atlas_b2p_hash_present_flag(bool value) -> auto &;
  auto daih_decoded_atlas_tiles_hash_present_flag(bool value) -> auto &;
  auto daih_decoded_atlas_tiles_b2p_hash_present_flag(bool value) -> auto &;
  auto decoded_high_level_hash(DecodedHighLevelHash &dhlh) -> auto &;
  auto decoded_atlas_hash(DecodedAtlasHash &dah) -> auto &;
  auto decoded_atlas_b2p_hash(DecodedAtlasB2pHash &dabh) -> auto &;
  auto daih_num_tiles_minus1(uint8_t value) -> auto &;
  auto daih_tile_id_len_minus1(uint8_t value) -> auto &;
  auto daih_tile_id(uint8_t tileId, uint8_t value) -> auto &;
  auto decoded_atlas_tile_hash(DecodedAtlasTileHash &dath) -> auto &;
  auto decoded_atlas_tile_b2p_hash(DecodedAtlasTileB2pHash &datbh) -> auto &;

  friend auto operator<<(std::ostream &stream, const DecodedAtlasInformationHash &x)
      -> std::ostream &;

  auto operator==(const DecodedAtlasInformationHash &other) const -> bool;
  auto operator!=(const DecodedAtlasInformationHash &other) const -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> DecodedAtlasInformationHash;
  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  bool m_daih_cancel_flag{true};
  std::optional<bool> m_daih_persistence_flag{};
  std::optional<uint8_t> m_daih_hash_type{};
  std::optional<bool> m_daih_decoded_high_level_hash_present_flag{};
  std::optional<bool> m_daih_decoded_atlas_hash_present_flag{};
  std::optional<bool> m_daih_decoded_atlas_b2p_hash_present_flag{};
  std::optional<bool> m_daih_decoded_atlas_tiles_hash_present_flag{};
  std::optional<bool> m_daih_decoded_atlas_tiles_b2p_hash_present_flag{};
  std::optional<DecodedHighLevelHash> m_decoded_high_level_hash;
  std::optional<DecodedAtlasHash> m_decoded_atlas_hash;
  std::optional<DecodedAtlasB2pHash> m_decoded_atlas_b2p_hash;
  std::optional<uint8_t> m_daih_num_tiles_minus1{};
  std::optional<uint8_t> m_daih_tile_id_len_minus1{};
  std::optional<std::vector<uint8_t>> m_daih_tile_id{};
  std::optional<DecodedAtlasTileHash> m_decoded_atlas_tile_hash;
  std::optional<DecodedAtlasTileB2pHash> m_decoded_atlas_tile_b2p_hash;
};

} // namespace TMIV::MivBitstream

#include "DecodedAtlasInformationHash.hpp"

#endif
