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

#ifndef _TMIV_IO_MPIPCS_H_
#define _TMIV_IO_MPIPCS_H_

#include <TMIV/IO/IO.h>

namespace TMIV::MpiPcs {
class FileHeader {
public:
  static void read(std::istream &stream);
  static void write(std::ostream &stream);

private:
  static constexpr std::array<char, 10> content{'M', 'P', 'I', '_', 'P', 'C', 'S', '_', '1', '\0'};
};

class Reader {
public:
  Reader() = default;
  Reader(const Common::Json &config, const IO::Placeholders &placeholders,
         const MivBitstream::SequenceConfig &sc, bool buildIndexOn = true);
  [[nodiscard]] auto getPath() const -> const std::filesystem::path & { return m_path; }
  auto read(std::istream &stream, std::int32_t posId, Common::Vec2i size) -> Common::MpiPcs::Frame;
  auto read(std::int32_t frameId) -> Common::MpiPcs::Frame;

private:
  std::filesystem::path m_path{};
  Common::Vec2i m_size{};
  std::vector<size_t> m_index{};
  int32_t m_startFrame{};
  void buildIndex();
};

class Writer {
public:
  Writer() = default;
  Writer(const Common::Json &config, const IO::Placeholders &placeholders,
         const MivBitstream::SequenceConfig &sc);
  [[nodiscard]] auto getPath() const -> const std::filesystem::path & { return m_path; }
  void append(std::ostream &stream, const Common::MpiPcs::Frame &mpiPcsFrame);
  void append(const Common::MpiPcs::Frame &mpiPcsFrame);

private:
  template <typename T> void writeToStream(std::ostream &stream, std::vector<T> &items) const;

  std::filesystem::path m_path{};
};

} // namespace TMIV::MpiPcs

#endif