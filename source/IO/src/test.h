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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_range.hpp>

#include <TMIV/IO/IO.h>

#include "DependencyInjector.h"
#include "FakeFilesystem.h"

using namespace std::string_literals;
using namespace std::string_view_literals;

namespace test {
using TMIV::Common::assertDownCast;
using TMIV::Common::ColorFormat;
using TMIV::Common::DefaultElement;
using TMIV::Common::Frame;
using TMIV::Common::Vec2i;
using TMIV::IO::DependencyInjector;
using TMIV::IO::Placeholders;

template <typename... Args>
auto injectFakeFilesystem(Args &&...args) -> std::shared_ptr<FakeFilesystem> {
  auto &injector = DependencyInjector::getInstance();
  auto filesystem = std::make_shared<FakeFilesystem>(std::forward<Args>(args)...);
  injector.filesystem(filesystem);
  return filesystem;
}

inline auto placeholders() {
  auto result = Placeholders{};

  result.contentId = "seq";
  result.numberOfInputFrames = 7;
  result.numberOfOutputFrames = 11;
  result.startFrame = 19;
  result.testId = "rate";

  return result;
}

inline auto frame(Vec2i size, ColorFormat colorFormat, uint32_t bitDepth) {
  auto frame = Frame<>{size, bitDepth, colorFormat};

  static constexpr auto exampleData = "The quick brown fox jumps over the lazy dog"sv;

  for (auto &plane : frame.getPlanes()) {
    const auto n = std::min(exampleData.size(), plane.size());

    std::transform(exampleData.cbegin(), exampleData.cbegin() + n, plane.begin(),
                   [](auto sample) { return assertDownCast<DefaultElement>(sample); });
  }

  return frame;
}

inline auto dir1() { return std::filesystem::path("fake"); }
inline auto dir2() { return std::filesystem::path("mirage"); }
} // namespace test
