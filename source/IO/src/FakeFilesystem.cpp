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

#include <catch2/catch.hpp>

#include "FakeFilesystem.h"

#include <TMIV/Common/Common.h>

#include <fmt/ostream.h>

namespace test {
using TMIV::Common::contains;

auto FakeFilesystem::ifstream(const std::filesystem::path &path, std::ios_base::openmode mode)
    -> std::shared_ptr<std::istream> {
  fmt::print("FakeFilesystem: ifstream {} mode {:x}\n", path, mode);

  REQUIRE(haveFile(path));
  return m_files[path];
}

auto FakeFilesystem::ofstream(const std::filesystem::path &path, std::ios_base::openmode mode)
    -> std::shared_ptr<std::ostream> {
  fmt::print("FakeFilesystem: ofstream {} mode {:x}\n", path, mode);

  REQUIRE_FALSE(haveFile(path));
  CHECK(contains(m_directories, path.parent_path()));

  auto file = std::make_shared<std::stringstream>();
  m_files[path] = file;
  return file;
}

void FakeFilesystem::create_directories(const std::filesystem::path &p) {
  fmt::print("FakeFilesystem: create_directories {}\n", p);

  if (!contains(m_directories, p)) {
    m_directories.push_back(p);
  }
}

auto FakeFilesystem::exists(const std::filesystem::path &p) -> bool {
  const auto result = haveFile(p) || haveDir(p);

  fmt::print("FakeFilesystem: exists {} -> {}\n", p, result);

  return result;
}

auto FakeFilesystem::empty() const noexcept -> bool {
  return m_files.empty() && m_directories.empty();
}

auto FakeFilesystem::haveFile(const std::filesystem::path &path) const noexcept -> bool {
  return std::any_of(m_files.cbegin(), m_files.cend(),
                     [&path](const auto &node) { return node.key == path; });
}

auto FakeFilesystem::haveDir(const std::filesystem::path &path) const noexcept -> bool {
  return contains(m_directories, path);
}

auto FakeFilesystem::fileData(const std::filesystem::path &path) const -> std::string {
  REQUIRE(haveFile(path));
  return m_files[path]->str();
}

void FakeFilesystem::fileData(const std::filesystem::path &path, const std::string &value) {
  m_files[path] = std::make_shared<std::stringstream>(value);
}
} // namespace test
