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

#ifndef TMIV_MIVBITSTREAM_DECODEDATLASINFORMATIONHASH_H
#define TMIV_MIVBITSTREAM_DECODEDATLASINFORMATIONHASH_H

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/FlatMap.h>

#include <array>
#include <optional>
#include <string_view>
#include <vector>

namespace TMIV::MivBitstream {
// 23090-5: *_hash
class Hash {
public:
  [[nodiscard]] auto md5(uint8_t i) const -> uint8_t;
  [[nodiscard]] auto crc() const -> uint16_t;
  [[nodiscard]] auto checksum() const -> uint32_t;

  auto md5(uint8_t i, uint8_t value) -> Hash &;
  auto crc(uint16_t value) -> Hash &;
  auto checksum(uint32_t value) -> Hash &;

  void printTo(std::string_view prefix, std::string_view index, std::ostream &stream,
               uint8_t hashType) const;

  [[nodiscard]] auto operator==(const Hash &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, uint8_t hashType) -> Hash;
  void encodeTo(Common::OutputBitstream &bitstream, uint8_t hashType) const;

private:
  std::optional<std::array<uint8_t, 16>> m_md5;
  std::optional<uint16_t> m_crc;
  std::optional<uint32_t> m_checksum;
};

// 23090-5: decoded_high_level_hash
class DecodedHighLevelHash {
public:
  [[nodiscard]] auto daih_high_level_md5(uint8_t i) const { return m_hash.md5(i); }
  [[nodiscard]] auto daih_high_level_crc() const { return m_hash.crc(); }
  [[nodiscard]] auto daih_high_level_checksum() const { return m_hash.checksum(); }

  auto daih_high_level_md5(uint8_t i, uint8_t value) { return m_hash.md5(i, value); }
  auto daih_high_level_crc(uint16_t value) { return m_hash.crc(value); }
  auto daih_high_level_checksum(uint32_t value) { m_hash.checksum(value); }

  void printTo(std::ostream &stream, uint8_t hashType) const {
    return m_hash.printTo("daih_high_level", {}, stream, hashType);
  }

  [[nodiscard]] auto operator==(const DecodedHighLevelHash &other) const -> bool {
    return m_hash == other.m_hash;
  };

  static auto decodeFrom(Common::InputBitstream &bitstream, uint8_t hashType) {
    return DecodedHighLevelHash{Hash::decodeFrom(bitstream, hashType)};
  };

  void encodeTo(Common::OutputBitstream &bitstream, uint8_t hashType) const {
    m_hash.encodeTo(bitstream, hashType);
  }

private:
  explicit DecodedHighLevelHash(const Hash &hash) : m_hash{hash} {}

  Hash m_hash;
};

// 23090-5: decoded_atlas_hash
class DecodedAtlasHash {
public:
  DecodedAtlasHash() = default;

  [[nodiscard]] auto daih_atlas_md5(uint8_t i) const { return m_hash.md5(i); }
  [[nodiscard]] auto daih_atlas_crc() const { return m_hash.crc(); }
  [[nodiscard]] auto daih_atlas_checksum() const { return m_hash.checksum(); }

  auto daih_atlas_md5(uint8_t i, uint8_t value) { return m_hash.md5(i, value); }
  auto daih_atlas_crc(uint16_t value) { return m_hash.crc(value); }
  auto daih_atlas_checksum(uint32_t value) { m_hash.checksum(value); }

  void printTo(std::ostream &stream, uint8_t hashType) const {
    return m_hash.printTo("daih_atlas", {}, stream, hashType);
  }

  [[nodiscard]] auto operator==(const DecodedAtlasHash &other) const noexcept -> bool {
    return m_hash == other.m_hash;
  };

  static auto decodeFrom(Common::InputBitstream &bitstream, uint8_t hashType) {
    return DecodedAtlasHash{Hash::decodeFrom(bitstream, hashType)};
  };

  void encodeTo(Common::OutputBitstream &bitstream, uint8_t hashType) const {
    m_hash.encodeTo(bitstream, hashType);
  }

private:
  explicit DecodedAtlasHash(const Hash &hash) : m_hash{hash} {}

  Hash m_hash;
};

// 23090-5: decoded_atlas_b2p_hash
class DecodedAtlasB2pHash {
public:
  DecodedAtlasB2pHash() = default;

  [[nodiscard]] auto daih_atlas_b2p_md5(uint8_t i) const { return m_hash.md5(i); }
  [[nodiscard]] auto daih_atlas_b2p_crc() const { return m_hash.crc(); }
  [[nodiscard]] auto daih_atlas_b2p_checksum() const { return m_hash.checksum(); }

  auto daih_atlas_b2p_md5(uint8_t i, uint8_t value) { return m_hash.md5(i, value); }
  auto daih_atlas_b2p_crc(uint16_t value) { return m_hash.crc(value); }
  auto daih_atlas_b2p_checksum(uint32_t value) { m_hash.checksum(value); }

  void printTo(std::ostream &stream, uint8_t hashType) const {
    return m_hash.printTo("daih_atlas_b2p", {}, stream, hashType);
  }

  [[nodiscard]] auto operator==(const DecodedAtlasB2pHash &other) const noexcept -> bool {
    return m_hash == other.m_hash;
  };

  static auto decodeFrom(Common::InputBitstream &bitstream, uint8_t hashType) {
    return DecodedAtlasB2pHash{Hash::decodeFrom(bitstream, hashType)};
  };

  void encodeTo(Common::OutputBitstream &bitstream, uint8_t hashType) const {
    m_hash.encodeTo(bitstream, hashType);
  }

private:
  explicit DecodedAtlasB2pHash(const Hash &hash) : m_hash{hash} {}

  Hash m_hash;
};

// 23090-5: decoded_atlas_tile_hash
class DecodedAtlasTileHash {
public:
  DecodedAtlasTileHash() = default;

  [[nodiscard]] auto daih_atlas_tile_md5(uint8_t i) const { return m_hash.md5(i); }
  [[nodiscard]] auto daih_atlas_tile_crc() const { return m_hash.crc(); }
  [[nodiscard]] auto daih_atlas_tile_checksum() const { return m_hash.checksum(); }

  auto daih_atlas_tile_md5(uint8_t i, uint8_t value) { return m_hash.md5(i, value); }
  auto daih_atlas_tile_crc(uint16_t value) { return m_hash.crc(value); }
  auto daih_atlas_tile_checksum(uint32_t value) { m_hash.checksum(value); }

  void printTo(std::ostream &stream, uint8_t hashType, uint8_t j) const;

  [[nodiscard]] auto operator==(const DecodedAtlasTileHash &other) const noexcept -> bool {
    return m_hash == other.m_hash;
  }

  static auto decodeFrom(Common::InputBitstream &bitstream, uint8_t hashType) {
    return DecodedAtlasTileHash{Hash::decodeFrom(bitstream, hashType)};
  }

  void encodeTo(Common::OutputBitstream &bitstream, uint8_t hashType) const {
    m_hash.encodeTo(bitstream, hashType);
  }

private:
  explicit DecodedAtlasTileHash(const Hash &hash) : m_hash{hash} {}

  Hash m_hash;
};

// 23090-5: decoded_atlas_tile_b2p_hash
class DecodedAtlasTileB2pHash {
public:
  DecodedAtlasTileB2pHash() = default;

  [[nodiscard]] auto daih_atlas_tile_b2p_md5(uint8_t i) const { return m_hash.md5(i); }
  [[nodiscard]] auto daih_atlas_tile_b2p_crc() const { return m_hash.crc(); }
  [[nodiscard]] auto daih_atlas_tile_b2p_checksum() const { return m_hash.checksum(); }

  auto daih_atlas_tile_b2p_md5(uint8_t i, uint8_t value) { return m_hash.md5(i, value); }
  auto daih_atlas_tile_b2p_crc(uint16_t value) { return m_hash.crc(value); }
  auto daih_atlas_tile_b2p_checksum(uint32_t value) { m_hash.checksum(value); }

  void printTo(std::ostream &stream, uint8_t hashType, uint8_t j) const;

  [[nodiscard]] auto operator==(const DecodedAtlasTileB2pHash &other) const noexcept -> bool {
    return m_hash == other.m_hash;
  }

  static auto decodeFrom(Common::InputBitstream &bitstream, uint8_t hashType) {
    return DecodedAtlasTileB2pHash{Hash::decodeFrom(bitstream, hashType)};
  }

  void encodeTo(Common::OutputBitstream &bitstream, uint8_t hashType) const {
    m_hash.encodeTo(bitstream, hashType);
  }

private:
  explicit DecodedAtlasTileB2pHash(const Hash &hash) : m_hash{hash} {}

  Hash m_hash;
};

// 23090-5: decoded_atlas_information_hash( payloadSize )
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
  [[nodiscard]] auto daih_tile_id(uint8_t t) const -> uint8_t;
  [[nodiscard]] auto decoded_atlas_tile_hash(uint8_t j) const -> const DecodedAtlasTileHash &;
  [[nodiscard]] auto decoded_atlas_tile_b2p_hash(uint8_t j) const
      -> const DecodedAtlasTileB2pHash &;

  auto daih_cancel_flag(bool value) -> DecodedAtlasInformationHash &;
  auto daih_persistence_flag(bool value) -> DecodedAtlasInformationHash &;
  auto daih_hash_type(uint8_t value) -> DecodedAtlasInformationHash &;
  auto daih_decoded_high_level_hash_present_flag(bool value) -> DecodedAtlasInformationHash &;
  auto daih_decoded_atlas_hash_present_flag(bool value) -> DecodedAtlasInformationHash &;
  auto daih_decoded_atlas_b2p_hash_present_flag(bool value) -> DecodedAtlasInformationHash &;
  auto daih_decoded_atlas_tiles_hash_present_flag(bool value) -> DecodedAtlasInformationHash &;
  auto daih_decoded_atlas_tiles_b2p_hash_present_flag(bool value) -> DecodedAtlasInformationHash &;
  auto decoded_high_level_hash(const DecodedHighLevelHash &value) -> DecodedAtlasInformationHash &;
  auto decoded_atlas_hash(const DecodedAtlasHash &value) -> DecodedAtlasInformationHash &;
  auto decoded_atlas_b2p_hash(const DecodedAtlasB2pHash &value) -> DecodedAtlasInformationHash &;
  auto daih_num_tiles_minus1(uint8_t value) -> DecodedAtlasInformationHash &;
  auto daih_tile_id_len_minus1(uint8_t value) -> DecodedAtlasInformationHash &;
  auto daih_tile_id(uint8_t tileId, uint8_t value) -> DecodedAtlasInformationHash &;
  auto decoded_atlas_tile_hash(uint8_t j, const DecodedAtlasTileHash &value)
      -> DecodedAtlasInformationHash &;
  auto decoded_atlas_tile_b2p_hash(uint8_t j, const DecodedAtlasTileB2pHash &value)
      -> DecodedAtlasInformationHash &;

  friend auto operator<<(std::ostream &stream, const DecodedAtlasInformationHash &x)
      -> std::ostream &;

  auto operator==(const DecodedAtlasInformationHash &other) const -> bool;
  auto operator!=(const DecodedAtlasInformationHash &other) const -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> DecodedAtlasInformationHash;
  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  [[nodiscard]] auto indexOfTileId(uint8_t j) const -> uint8_t;

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
  std::vector<uint8_t> m_daih_tile_id{};                              // indexed by t
  std::vector<DecodedAtlasTileHash> m_decoded_atlas_tile_hash;        // indexed by t
  std::vector<DecodedAtlasTileB2pHash> m_decoded_atlas_tile_b2p_hash; // indexed by t
};

} // namespace TMIV::MivBitstream

#endif
