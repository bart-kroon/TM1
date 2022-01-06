/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#include <fmt/ostream.h>

namespace TMIV::MivBitstream {
auto operator<<(std::ostream &stream, const DecodedAtlasHash &x) -> std::ostream & {
  if (x.m_daih_atlas_md5.has_value()) {
    stream << "daih_atlas_md5=";
    for (uint8_t i = 0; i < 16; i++) {
      stream << fmt::format(FMT_STRING("{:02x}"), x.daih_atlas_md5(i));
    }
    stream << "\n";
  } else if (x.m_daih_atlas_crc.has_value()) {
    stream << "daih_atlas_crc=" << fmt::format(FMT_STRING("{:04x}"), x.daih_atlas_crc()) << "\n";
  } else if (x.m_daih_atlas_checksum.has_value()) {
    stream << "daih_atlas_checksum=" << fmt::format(FMT_STRING("{:08x}"), x.daih_atlas_checksum())
           << "\n";
  }
  return stream;
}

auto operator<<(std::ostream &stream, const DecodedAtlasInformationHash &x) -> std::ostream & {
  fmt::print(stream, "daih_cancel_flag={}\n", x.daih_cancel_flag());
  if (!x.daih_cancel_flag()) {
    fmt::print(stream, "daih_persistence_flag={}\n", x.daih_persistence_flag());
    fmt::print(stream, "daih_hash_type={}\n", x.daih_hash_type());
    fmt::print(stream, "daih_decoded_high_level_hash_present_flag={}\n",
               x.daih_decoded_high_level_hash_present_flag());
    fmt::print(stream, "daih_decoded_atlas_hash_present_flag={}\n",
               x.daih_decoded_atlas_hash_present_flag());
    fmt::print(stream, "daih_decoded_atlas_b2p_hash_present_flag={}\n",
               x.daih_decoded_atlas_b2p_hash_present_flag());
    fmt::print(stream, "daih_decoded_atlas_tiles_hash_present_flag={}\n",
               x.daih_decoded_atlas_tiles_hash_present_flag());
    fmt::print(stream, "daih_decoded_atlas_tiles_b2p_hash_present_flag={}\n",
               x.daih_decoded_atlas_tiles_b2p_hash_present_flag());
    if (x.daih_decoded_high_level_hash_present_flag()) {
      stream << x.decoded_high_level_hash();
    }
    if (x.daih_decoded_atlas_hash_present_flag()) {
      stream << x.decoded_atlas_hash();
    }
    if (x.daih_decoded_atlas_b2p_hash_present_flag()) {
      stream << x.decoded_atlas_b2p_hash();
    }
    if (x.daih_decoded_atlas_tiles_hash_present_flag() ||
        x.daih_decoded_atlas_tiles_b2p_hash_present_flag()) {
      fmt::print(stream, "daih_num_tiles_minus1={}\n", x.daih_num_tiles_minus1());
      fmt::print(stream, "daih_tile_id_len_minus1={}\n", x.daih_tile_id_len_minus1());
      for (uint8_t t = 0; t <= x.daih_num_tiles_minus1(); t++) {
        fmt::print(stream, "daih_tile_id[ {} ]={}\n", x.daih_tile_id(t));
      }
      if (x.daih_decoded_atlas_tiles_hash_present_flag()) {
        stream << x.decoded_atlas_tile_hash();
      }
      if (x.daih_decoded_atlas_tiles_b2p_hash_present_flag()) {
        stream << x.decoded_atlas_tile_b2p_hash();
      }
    }
  }
  return stream;
}

auto DecodedAtlasHash::operator==(const DecodedAtlasHash &other) const -> bool {
  bool isEqual = true;
  if (m_daih_atlas_md5.has_value()) {
    for (uint8_t i = 0; i < 16; i++) {
      if (daih_atlas_md5(i) != other.daih_atlas_md5(i)) {
        isEqual = false;
      }
    }
  } else if (m_daih_atlas_crc.has_value()) {
    if (daih_atlas_crc() != other.daih_atlas_crc()) {
      isEqual = false;
    }
  } else if (m_daih_atlas_checksum.has_value()) {
    if (daih_atlas_checksum() != other.daih_atlas_checksum()) {
      isEqual = false;
    }
  }
  return isEqual;
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

void DecodedAtlasHash::encodeTo(Common::OutputBitstream &bitstream, uint8_t hashType) const {
  if (hashType == 0) {
    for (uint8_t i = 0; i < 16; i++) {
      bitstream.writeBits(daih_atlas_md5(i), 8);
    }
  } else if (hashType == 1) {
    bitstream.putUint16(daih_atlas_crc());
  } else if (hashType == 2) {
    bitstream.putUint32(daih_atlas_checksum());
  }
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
    constexpr auto daih_reserved_zero_1bit = 0;
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
      for (uint8_t t = 0; t <= daih_num_tiles_minus1(); t++) {
        bitstream.putUVar(daih_tile_id(t), daih_num_tiles_minus1() + 1);
      }
      bitstream.byteAlignment();
      for (uint8_t t = 0; t <= daih_num_tiles_minus1(); t++) {
        auto j = daih_tile_id(t);
        if (daih_decoded_atlas_tiles_hash_present_flag()) {
          decoded_atlas_tile_hash().encodeTo(bitstream, daih_hash_type(), j);
        }
        if (daih_decoded_atlas_tiles_b2p_hash_present_flag()) {
          decoded_atlas_tile_b2p_hash().encodeTo(bitstream, daih_hash_type(), j);
        }
      }
    }
  }
}

auto DecodedAtlasHash::decodeFrom(Common::InputBitstream &bitstream, uint8_t hashType)
    -> DecodedAtlasHash {
  DecodedAtlasHash result{};
  if (hashType == 0) {
    for (uint8_t i = 0; i < 16; i++) {
      result.daih_atlas_md5(i, bitstream.readBits<uint8_t>(8));
    }
  } else if (hashType == 1) {
    result.daih_atlas_crc(bitstream.getUint16());
  } else if (hashType == 2) {
    result.daih_atlas_checksum(bitstream.getUint32());
  }
  return result;
}

auto DecodedAtlasInformationHash::decodeFrom(Common::InputBitstream &bitstream)
    -> DecodedAtlasInformationHash {
  DecodedAtlasInformationHash result{};
  result.daih_cancel_flag(bitstream.getFlag());
  if (!result.daih_cancel_flag()) {
    result.daih_persistence_flag(bitstream.getFlag());
    result.daih_hash_type(bitstream.getUint8());
    result.daih_decoded_high_level_hash_present_flag(bitstream.getFlag());
    result.daih_decoded_atlas_hash_present_flag(bitstream.getFlag());
    result.daih_decoded_atlas_b2p_hash_present_flag(bitstream.getFlag());
    result.daih_decoded_atlas_tiles_hash_present_flag(bitstream.getFlag());
    result.daih_decoded_atlas_tiles_b2p_hash_present_flag(bitstream.getFlag());
    bitstream.readBits<uint8_t>(1);
    if (result.daih_decoded_high_level_hash_present_flag()) {
      result.m_decoded_high_level_hash =
          DecodedHighLevelHash::decodeFrom(bitstream, result.daih_hash_type());
    }
    if (result.daih_decoded_atlas_hash_present_flag()) {
      result.m_decoded_atlas_hash =
          DecodedAtlasHash::decodeFrom(bitstream, result.daih_hash_type());
    }
    if (result.daih_decoded_atlas_b2p_hash_present_flag()) {
      result.m_decoded_atlas_b2p_hash =
          DecodedAtlasB2pHash::decodeFrom(bitstream, result.daih_hash_type());
    }
    if (result.daih_decoded_atlas_tiles_hash_present_flag() ||
        result.daih_decoded_atlas_tiles_b2p_hash_present_flag()) {
      result.daih_num_tiles_minus1(bitstream.getUExpGolomb<uint8_t>());
      result.daih_tile_id_len_minus1(bitstream.getUExpGolomb<uint8_t>());
      for (uint8_t t = 0; t <= result.daih_num_tiles_minus1(); t++) {
        result.daih_tile_id(t, bitstream.getUVar<uint8_t>(result.daih_num_tiles_minus1() + 1));
      }
      bitstream.byteAlignment();
      for (uint8_t t = 0; t <= result.daih_num_tiles_minus1(); t++) {
        auto j = result.daih_tile_id(t);
        if (result.daih_decoded_atlas_tiles_hash_present_flag()) {
          result.m_decoded_atlas_tile_hash =
              DecodedAtlasTileHash::decodeFrom(bitstream, result.daih_hash_type(), j);
        }
        if (result.daih_decoded_atlas_tiles_b2p_hash_present_flag()) {
          result.m_decoded_atlas_tile_b2p_hash =
              DecodedAtlasTileB2pHash::decodeFrom(bitstream, result.daih_hash_type(), j);
        }
      }
    }
  }
  return result;
}
} // namespace TMIV::MivBitstream
