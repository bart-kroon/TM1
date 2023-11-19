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

#include <TMIV/MivBitstream/DecodedAtlasInformationHash.h>

#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/Formatters.h>

namespace TMIV::MivBitstream {
auto Hash::md5(uint8_t i) const -> uint8_t {
  VERIFY_V3CBITSTREAM(m_md5.has_value());
  return Common::at(*m_md5, i);
}

auto Hash::crc() const -> uint16_t {
  VERIFY_V3CBITSTREAM(m_crc.has_value());
  return *m_crc;
}

auto Hash::checksum() const -> uint32_t {
  VERIFY_V3CBITSTREAM(m_checksum.has_value());
  return *m_checksum;
}

auto Hash::md5(uint8_t i, uint8_t value) -> Hash & {
  if (!m_md5.has_value()) {
    m_md5 = std::array<uint8_t, 16>{};
  }
  Common::at(*m_md5, i) = value;
  return *this;
}

auto Hash::crc(uint16_t value) -> Hash & {
  m_crc = value;
  return *this;
}

auto Hash::checksum(uint32_t value) -> Hash & {
  m_checksum = value;
  return *this;
}

void Hash::printTo(std::string_view prefix, std::string_view index, std::ostream &stream,
                   uint8_t hashType) const {
  switch (hashType) {
  case 0:
    TMIV_FMT::print(stream, "{}_md5{}=", prefix, index);

    for (uint8_t i = 0; i < 16; ++i) {
      TMIV_FMT::print(stream, "{:02x}", md5(i));
    }
    TMIV_FMT::print(stream, " (hex)\n");
    return;
  case 1:
    return TMIV_FMT::print(stream, "{}_crc{}={:04x} (hex)\n", prefix, index, crc());
  case 2:
    return TMIV_FMT::print(stream, "{}_checksum{}={:08x} (hex)\n", prefix, index, checksum());
  }
}

auto Hash::operator==(const Hash &other) const noexcept -> bool {
  return m_md5 == other.m_md5 && m_crc == other.m_crc && m_checksum == other.m_checksum;
}

auto Hash::decodeFrom(Common::InputBitstream &bitstream, uint8_t hashType) -> Hash {
  auto x = Hash{};

  switch (hashType) {
  case 0: {
    for (uint8_t i = 0; i < 16; ++i) {
      x.md5(i, bitstream.getUint8());
    }
    break;
  }
  case 1:
    x.crc(bitstream.getUint16());
    break;
  case 2:
    x.checksum(bitstream.getUint32());
    break;
  default:
    V3CBITSTREAM_ERROR("Unknown hashType");
  }
  return x;
}

void Hash::encodeTo(Common::OutputBitstream &bitstream, uint8_t hashType) const {
  switch (hashType) {
  case 0:
    for (uint8_t i = 0; i < 16; ++i) {
      bitstream.putUint8(md5(i));
    }
    return;
  case 1:
    return bitstream.putUint16(crc());
  case 2:
    return bitstream.putUint32(checksum());
  default:
    PRECONDITION("Unknown hashType");
  }
}

void DecodedAtlasTileHash::printTo(std::ostream &stream, uint8_t hashType, uint8_t j) const {
  return m_hash.printTo("daih_atlas_tile", TMIV_FMT::format("[ {} ]", j), stream, hashType);
}

void DecodedAtlasTileB2pHash::printTo(std::ostream &stream, uint8_t hashType, uint8_t j) const {
  return m_hash.printTo("daih_atlas_tile_b2p", TMIV_FMT::format("[ {} ]", j), stream, hashType);
}

auto DecodedAtlasInformationHash::daih_cancel_flag() const -> bool { return m_daih_cancel_flag; }

auto DecodedAtlasInformationHash::daih_persistence_flag() const -> bool {
  VERIFY_V3CBITSTREAM(!daih_cancel_flag());
  VERIFY_V3CBITSTREAM(m_daih_persistence_flag.has_value());
  return *m_daih_persistence_flag;
}

auto DecodedAtlasInformationHash::daih_hash_type() const -> uint8_t {
  VERIFY_V3CBITSTREAM(!daih_cancel_flag());
  VERIFY_V3CBITSTREAM(m_daih_hash_type.has_value());
  return *m_daih_hash_type;
}

auto DecodedAtlasInformationHash::daih_decoded_high_level_hash_present_flag() const -> bool {
  VERIFY_V3CBITSTREAM(!daih_cancel_flag());
  VERIFY_V3CBITSTREAM(m_daih_decoded_high_level_hash_present_flag.has_value());
  return *m_daih_decoded_high_level_hash_present_flag;
}

auto DecodedAtlasInformationHash::daih_decoded_atlas_hash_present_flag() const -> bool {
  VERIFY_V3CBITSTREAM(!daih_cancel_flag());
  VERIFY_V3CBITSTREAM(m_daih_decoded_atlas_hash_present_flag.has_value());
  return *m_daih_decoded_atlas_hash_present_flag;
}

auto DecodedAtlasInformationHash::daih_decoded_atlas_b2p_hash_present_flag() const -> bool {
  VERIFY_V3CBITSTREAM(!daih_cancel_flag());
  VERIFY_V3CBITSTREAM(m_daih_decoded_atlas_b2p_hash_present_flag.has_value());
  return *m_daih_decoded_atlas_b2p_hash_present_flag;
}

auto DecodedAtlasInformationHash::daih_decoded_atlas_tiles_hash_present_flag() const -> bool {
  VERIFY_V3CBITSTREAM(!daih_cancel_flag());
  VERIFY_V3CBITSTREAM(m_daih_decoded_atlas_tiles_hash_present_flag.has_value());
  return *m_daih_decoded_atlas_tiles_hash_present_flag;
}

auto DecodedAtlasInformationHash::daih_decoded_atlas_tiles_b2p_hash_present_flag() const -> bool {
  VERIFY_V3CBITSTREAM(!daih_cancel_flag());
  VERIFY_V3CBITSTREAM(m_daih_decoded_atlas_tiles_b2p_hash_present_flag.has_value());
  return *m_daih_decoded_atlas_tiles_b2p_hash_present_flag;
}

auto DecodedAtlasInformationHash::decoded_high_level_hash() const -> const DecodedHighLevelHash & {
  VERIFY_V3CBITSTREAM(daih_decoded_high_level_hash_present_flag());
  VERIFY_V3CBITSTREAM(m_decoded_high_level_hash.has_value());
  return *m_decoded_high_level_hash;
}

auto DecodedAtlasInformationHash::decoded_atlas_hash() const -> const DecodedAtlasHash & {
  VERIFY_V3CBITSTREAM(daih_decoded_atlas_hash_present_flag());
  VERIFY_V3CBITSTREAM(m_decoded_atlas_hash.has_value());
  return *m_decoded_atlas_hash;
}

auto DecodedAtlasInformationHash::decoded_atlas_b2p_hash() const -> const DecodedAtlasB2pHash & {
  VERIFY_V3CBITSTREAM(daih_decoded_atlas_b2p_hash_present_flag());
  VERIFY_V3CBITSTREAM(m_decoded_atlas_b2p_hash.has_value());
  return *m_decoded_atlas_b2p_hash;
}

auto DecodedAtlasInformationHash::daih_num_tiles_minus1() const -> uint8_t {
  VERIFY_V3CBITSTREAM(daih_decoded_atlas_tiles_hash_present_flag() ||
                      daih_decoded_atlas_tiles_b2p_hash_present_flag());
  VERIFY_V3CBITSTREAM(m_daih_num_tiles_minus1.has_value());
  return *m_daih_num_tiles_minus1;
}

auto DecodedAtlasInformationHash::daih_tile_id_len_minus1() const -> uint8_t {
  VERIFY_V3CBITSTREAM(daih_decoded_atlas_tiles_hash_present_flag() ||
                      daih_decoded_atlas_tiles_b2p_hash_present_flag());
  VERIFY_V3CBITSTREAM(m_daih_tile_id_len_minus1.has_value());
  return *m_daih_tile_id_len_minus1;
}

auto DecodedAtlasInformationHash::daih_tile_id(uint8_t t) const -> uint8_t {
  VERIFY_V3CBITSTREAM(daih_decoded_atlas_tiles_hash_present_flag() ||
                      daih_decoded_atlas_tiles_b2p_hash_present_flag());
  VERIFY_V3CBITSTREAM(t < m_daih_tile_id.size());
  return m_daih_tile_id[t];
}

auto DecodedAtlasInformationHash::decoded_atlas_tile_hash(uint8_t j) const
    -> const DecodedAtlasTileHash & {
  VERIFY_V3CBITSTREAM(daih_decoded_atlas_tiles_hash_present_flag());
  VERIFY_V3CBITSTREAM(m_decoded_atlas_tile_hash.size() == daih_num_tiles_minus1() + size_t{1});
  return m_decoded_atlas_tile_hash[indexOfTileId(j)];
}

auto DecodedAtlasInformationHash::decoded_atlas_tile_b2p_hash(uint8_t j) const
    -> const DecodedAtlasTileB2pHash & {
  VERIFY_V3CBITSTREAM(daih_decoded_atlas_tiles_b2p_hash_present_flag());
  VERIFY_V3CBITSTREAM(m_decoded_atlas_tile_b2p_hash.size() == daih_num_tiles_minus1() + size_t{1});
  return m_decoded_atlas_tile_b2p_hash[indexOfTileId(j)];
}

auto DecodedAtlasInformationHash::daih_cancel_flag(bool value) -> DecodedAtlasInformationHash & {
  m_daih_cancel_flag = value;
  return *this;
}

auto DecodedAtlasInformationHash::daih_persistence_flag(bool value)
    -> DecodedAtlasInformationHash & {
  daih_cancel_flag(false);
  m_daih_persistence_flag = value;
  return *this;
}

auto DecodedAtlasInformationHash::daih_hash_type(uint8_t value) -> DecodedAtlasInformationHash & {
  daih_cancel_flag(false);
  m_daih_hash_type = value;
  return *this;
}

auto DecodedAtlasInformationHash::daih_decoded_high_level_hash_present_flag(bool value)
    -> DecodedAtlasInformationHash & {
  daih_cancel_flag(false);
  m_daih_decoded_high_level_hash_present_flag = value;
  return *this;
}

auto DecodedAtlasInformationHash::daih_decoded_atlas_hash_present_flag(bool value)
    -> DecodedAtlasInformationHash & {
  daih_cancel_flag(false);
  m_daih_decoded_atlas_hash_present_flag = value;
  return *this;
}

auto DecodedAtlasInformationHash::daih_decoded_atlas_b2p_hash_present_flag(bool value)
    -> DecodedAtlasInformationHash & {
  daih_cancel_flag(false);
  m_daih_decoded_atlas_b2p_hash_present_flag = value;
  return *this;
}

auto DecodedAtlasInformationHash::daih_decoded_atlas_tiles_hash_present_flag(bool value)
    -> DecodedAtlasInformationHash & {
  daih_cancel_flag(false);
  m_daih_decoded_atlas_tiles_hash_present_flag = value;
  return *this;
}

auto DecodedAtlasInformationHash::daih_decoded_atlas_tiles_b2p_hash_present_flag(bool value)
    -> DecodedAtlasInformationHash & {
  daih_cancel_flag(false);
  m_daih_decoded_atlas_tiles_b2p_hash_present_flag = value;
  return *this;
}

auto DecodedAtlasInformationHash::decoded_high_level_hash(const DecodedHighLevelHash &value)
    -> DecodedAtlasInformationHash & {
  daih_decoded_high_level_hash_present_flag(true);
  m_decoded_high_level_hash = value;
  return *this;
}

auto DecodedAtlasInformationHash::decoded_atlas_hash(const DecodedAtlasHash &value)
    -> DecodedAtlasInformationHash & {
  daih_decoded_atlas_hash_present_flag(true);
  m_decoded_atlas_hash = value;
  return *this;
}

auto DecodedAtlasInformationHash::decoded_atlas_b2p_hash(const DecodedAtlasB2pHash &value)
    -> DecodedAtlasInformationHash & {
  daih_decoded_atlas_b2p_hash_present_flag(true);
  m_decoded_atlas_b2p_hash = value;
  return *this;
}

auto DecodedAtlasInformationHash::daih_num_tiles_minus1(uint8_t value)
    -> DecodedAtlasInformationHash & {
  VERIFY_V3CBITSTREAM(daih_decoded_atlas_tiles_hash_present_flag() ||
                      daih_decoded_atlas_tiles_b2p_hash_present_flag());
  m_daih_num_tiles_minus1 = value;
  return *this;
}

auto DecodedAtlasInformationHash::daih_tile_id_len_minus1(uint8_t value)
    -> DecodedAtlasInformationHash & {
  VERIFY_V3CBITSTREAM(daih_decoded_atlas_tiles_hash_present_flag() ||
                      daih_decoded_atlas_tiles_b2p_hash_present_flag());
  m_daih_tile_id_len_minus1 = value;
  return *this;
}

auto DecodedAtlasInformationHash::daih_tile_id(uint8_t t, uint8_t value)
    -> DecodedAtlasInformationHash & {
  VERIFY_V3CBITSTREAM(t <= daih_num_tiles_minus1());
  m_daih_tile_id.resize(daih_num_tiles_minus1() + size_t{1});
  m_daih_tile_id[t] = value;
  return *this;
}

auto DecodedAtlasInformationHash::decoded_atlas_tile_hash(uint8_t j,
                                                          const DecodedAtlasTileHash &value)
    -> DecodedAtlasInformationHash & {
  daih_decoded_atlas_tiles_hash_present_flag(true);
  m_decoded_atlas_tile_hash.resize(daih_num_tiles_minus1() + size_t{1});
  m_decoded_atlas_tile_hash[indexOfTileId(j)] = value;
  return *this;
}

auto DecodedAtlasInformationHash::decoded_atlas_tile_b2p_hash(uint8_t j,
                                                              const DecodedAtlasTileB2pHash &value)
    -> DecodedAtlasInformationHash & {
  daih_decoded_atlas_tiles_b2p_hash_present_flag(true);
  m_decoded_atlas_tile_b2p_hash.resize(daih_num_tiles_minus1() + size_t{1});
  m_decoded_atlas_tile_b2p_hash[indexOfTileId(j)] = value;
  return *this;
}

auto operator<<(std::ostream &stream, const DecodedAtlasInformationHash &x) -> std::ostream & {
  TMIV_FMT::print(stream, "daih_cancel_flag={}\n", x.daih_cancel_flag());

  if (!x.daih_cancel_flag()) {
    TMIV_FMT::print(stream, "daih_persistence_flag={}\n", x.daih_persistence_flag());
    TMIV_FMT::print(stream, "daih_hash_type={}\n", x.daih_hash_type());
    TMIV_FMT::print(stream, "daih_decoded_high_level_hash_present_flag={}\n",
                    x.daih_decoded_high_level_hash_present_flag());
    TMIV_FMT::print(stream, "daih_decoded_atlas_hash_present_flag={}\n",
                    x.daih_decoded_atlas_hash_present_flag());
    TMIV_FMT::print(stream, "daih_decoded_atlas_b2p_hash_present_flag={}\n",
                    x.daih_decoded_atlas_b2p_hash_present_flag());
    TMIV_FMT::print(stream, "daih_decoded_atlas_tiles_hash_present_flag={}\n",
                    x.daih_decoded_atlas_tiles_hash_present_flag());
    TMIV_FMT::print(stream, "daih_decoded_atlas_tiles_b2p_hash_present_flag={}\n",
                    x.daih_decoded_atlas_tiles_b2p_hash_present_flag());

    if (x.daih_decoded_high_level_hash_present_flag()) {
      x.decoded_high_level_hash().printTo(stream, x.daih_hash_type());
    }
    if (x.daih_decoded_atlas_hash_present_flag()) {
      x.decoded_atlas_hash().printTo(stream, x.daih_hash_type());
    }
    if (x.daih_decoded_atlas_b2p_hash_present_flag()) {
      x.decoded_atlas_b2p_hash().printTo(stream, x.daih_hash_type());
    }
    if (x.daih_decoded_atlas_tiles_hash_present_flag() ||
        x.daih_decoded_atlas_tiles_b2p_hash_present_flag()) {
      TMIV_FMT::print(stream, "daih_num_tiles_minus1={}\n", x.daih_num_tiles_minus1());
      TMIV_FMT::print(stream, "daih_tile_id_len_minus1={}\n", x.daih_tile_id_len_minus1());

      for (uint8_t t = 0; t <= x.daih_num_tiles_minus1(); ++t) {
        TMIV_FMT::print(stream, "daih_tile_id[ {} ]={}\n", t, x.daih_tile_id(t));
      }
      for (uint8_t t = 0; t <= x.daih_num_tiles_minus1(); ++t) {
        const auto j = x.daih_tile_id(t);

        if (x.daih_decoded_atlas_tiles_hash_present_flag()) {
          x.decoded_atlas_tile_hash(j).printTo(stream, x.daih_hash_type(), j);
        }
        if (x.daih_decoded_atlas_tiles_b2p_hash_present_flag()) {
          x.decoded_atlas_tile_b2p_hash(j).printTo(stream, x.daih_hash_type(), j);
        }
      }
    }
  }
  return stream;
}

auto DecodedAtlasInformationHash::operator==(const DecodedAtlasInformationHash &other) const
    -> bool {
  if (daih_cancel_flag()) {
    return daih_cancel_flag() == other.daih_cancel_flag();
  }
  return daih_cancel_flag() == other.daih_cancel_flag() &&
         daih_persistence_flag() == other.daih_persistence_flag() &&
         daih_hash_type() == other.daih_hash_type() &&
         daih_decoded_high_level_hash_present_flag() ==
             other.daih_decoded_high_level_hash_present_flag() &&
         daih_decoded_atlas_hash_present_flag() == other.daih_decoded_atlas_hash_present_flag() &&
         daih_decoded_atlas_b2p_hash_present_flag() ==
             other.daih_decoded_atlas_b2p_hash_present_flag() &&
         daih_decoded_atlas_tiles_hash_present_flag() ==
             other.daih_decoded_atlas_tiles_hash_present_flag() &&
         daih_decoded_atlas_tiles_b2p_hash_present_flag() ==
             other.daih_decoded_atlas_tiles_b2p_hash_present_flag() &&
         decoded_atlas_hash() == other.decoded_atlas_hash();
}

auto DecodedAtlasInformationHash::operator!=(const DecodedAtlasInformationHash &other) const
    -> bool {
  return !operator==(other);
}

void DecodedAtlasInformationHash::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putFlag(daih_cancel_flag());

  if (!daih_cancel_flag()) {
    bitstream.putFlag(daih_persistence_flag());
    bitstream.putUint8(daih_hash_type());
    bitstream.putFlag(daih_decoded_high_level_hash_present_flag());
    bitstream.putFlag(daih_decoded_atlas_hash_present_flag());
    bitstream.putFlag(daih_decoded_atlas_b2p_hash_present_flag());
    bitstream.putFlag(daih_decoded_atlas_tiles_hash_present_flag());
    bitstream.putFlag(daih_decoded_atlas_tiles_b2p_hash_present_flag());

    static constexpr auto daih_reserved_zero_1bit = 0;
    bitstream.writeBits(daih_reserved_zero_1bit, 1);

    if (daih_decoded_high_level_hash_present_flag()) {
      decoded_high_level_hash().encodeTo(bitstream, daih_hash_type());
    }
    if (daih_decoded_atlas_hash_present_flag()) {
      decoded_atlas_hash().encodeTo(bitstream, daih_hash_type());
    }
    if (daih_decoded_atlas_b2p_hash_present_flag()) {
      decoded_atlas_b2p_hash().encodeTo(bitstream, daih_hash_type());
    }
    if (daih_decoded_atlas_tiles_hash_present_flag() ||
        daih_decoded_atlas_tiles_b2p_hash_present_flag()) {
      bitstream.putUExpGolomb(daih_num_tiles_minus1());
      bitstream.putUExpGolomb(daih_tile_id_len_minus1());

      for (uint8_t t = 0; t <= daih_num_tiles_minus1(); ++t) {
        bitstream.writeBits(daih_tile_id(t), daih_tile_id_len_minus1() + 1);
      }
      bitstream.byteAlignment();

      for (uint8_t t = 0; t <= daih_num_tiles_minus1(); ++t) {
        const auto j = daih_tile_id(t);

        if (daih_decoded_atlas_tiles_hash_present_flag()) {
          decoded_atlas_tile_hash(j).encodeTo(bitstream, daih_hash_type());
        }
        if (daih_decoded_atlas_tiles_b2p_hash_present_flag()) {
          decoded_atlas_tile_b2p_hash(j).encodeTo(bitstream, daih_hash_type());
        }
      }
    }
  }
}

auto DecodedAtlasInformationHash::decodeFrom(Common::InputBitstream &bitstream)
    -> DecodedAtlasInformationHash {
  auto x = DecodedAtlasInformationHash{};

  x.daih_cancel_flag(bitstream.getFlag());

  if (!x.daih_cancel_flag()) {
    x.daih_persistence_flag(bitstream.getFlag());
    x.daih_hash_type(bitstream.getUint8());
    x.daih_decoded_high_level_hash_present_flag(bitstream.getFlag());
    x.daih_decoded_atlas_hash_present_flag(bitstream.getFlag());
    x.daih_decoded_atlas_b2p_hash_present_flag(bitstream.getFlag());
    x.daih_decoded_atlas_tiles_hash_present_flag(bitstream.getFlag());
    x.daih_decoded_atlas_tiles_b2p_hash_present_flag(bitstream.getFlag());
    [[maybe_unused]] const auto daih_reserved_zero_1bit = bitstream.readBits<uint8_t>(1);

    if (x.daih_decoded_high_level_hash_present_flag()) {
      x.decoded_high_level_hash(DecodedHighLevelHash::decodeFrom(bitstream, x.daih_hash_type()));
    }
    if (x.daih_decoded_atlas_hash_present_flag()) {
      x.decoded_atlas_hash(DecodedAtlasHash::decodeFrom(bitstream, x.daih_hash_type()));
    }
    if (x.daih_decoded_atlas_b2p_hash_present_flag()) {
      x.decoded_atlas_b2p_hash(DecodedAtlasB2pHash::decodeFrom(bitstream, x.daih_hash_type()));
    }
    if (x.daih_decoded_atlas_tiles_hash_present_flag() ||
        x.daih_decoded_atlas_tiles_b2p_hash_present_flag()) {
      x.daih_num_tiles_minus1(bitstream.getUExpGolomb<uint8_t>());
      x.daih_tile_id_len_minus1(bitstream.getUExpGolomb<uint8_t>());

      for (uint8_t t = 0; t <= x.daih_num_tiles_minus1(); ++t) {
        x.daih_tile_id(t, bitstream.readBits<uint8_t>(x.daih_tile_id_len_minus1() + 1));
      }
      bitstream.byteAlignment();

      for (uint8_t t = 0; t <= x.daih_num_tiles_minus1(); ++t) {
        const auto j = x.daih_tile_id(t);

        if (x.daih_decoded_atlas_tiles_hash_present_flag()) {
          x.decoded_atlas_tile_hash(
              j, DecodedAtlasTileHash::decodeFrom(bitstream, x.daih_hash_type()));
        }
        if (x.daih_decoded_atlas_tiles_b2p_hash_present_flag()) {
          x.decoded_atlas_tile_b2p_hash(
              j, DecodedAtlasTileB2pHash::decodeFrom(bitstream, x.daih_hash_type()));
        }
      }
    }
  }
  return x;
}

auto DecodedAtlasInformationHash::indexOfTileId(uint8_t j) const -> uint8_t {
  for (uint8_t t = 0; t <= daih_num_tiles_minus1(); ++t) {
    if (j == daih_tile_id(t)) {
      return t;
    }
  }
  V3CBITSTREAM_ERROR("Unknown tile ID");
}
} // namespace TMIV::MivBitstream
