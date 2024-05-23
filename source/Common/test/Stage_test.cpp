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

#include <catch2/catch_test_macros.hpp>

#include <TMIV/Common/Stage.h>

namespace TMIV::Common::test {
template <typename In> struct FakeStageSink final : public IStageSink<In> {
  void encode(In unit) final { received.push_back(unit); }

  void flush() final { ++flushCount; }

  std::vector<In> received;
  size_t flushCount{};
};

template <typename In, typename Out>
struct FakeBufferingStage final : public BufferingStage<In, Out> {
  [[nodiscard]] auto isStart(const In &unit) -> bool final { return unit == 0; }

  void process(std::vector<In> buffer) final {
    for (auto unit : buffer) {
      this->source.encode(unit);
    }
  }
};
} // namespace TMIV::Common::test

namespace test = TMIV::Common::test;

using TMIV::Common::BufferingStage;
using TMIV::Common::StageSource;

TEST_CASE("TMIV::Common::StageSource") {
  auto unit = StageSource<int32_t>{};

  auto sink = test::FakeStageSink<int32_t>{};

  unit.connectTo(sink);

  REQUIRE(sink.received.empty());
  REQUIRE(sink.flushCount == 0);

  SECTION("encode") {
    unit.encode(1);
    unit.encode(2);
    unit.encode(3);

    REQUIRE(sink.received == std::vector{1, 2, 3});
  }

  SECTION("flush") {
    unit.flush();

    REQUIRE(sink.flushCount == 1);
  }
}

TEST_CASE("TMIV::Common::BufferingStage") {
  auto unit = test::FakeBufferingStage<int16_t, int32_t>{};

  auto sink = test::FakeStageSink<int32_t>{};

  unit.source.connectTo(sink);

  REQUIRE(sink.received.empty());
  REQUIRE(sink.flushCount == 0);

  unit.encode(1);
  unit.encode(2);
  unit.encode(3);

  REQUIRE(sink.received.empty());

  unit.encode(0);

  REQUIRE(sink.received == std::vector{1, 2, 3});

  unit.encode(4);
  unit.encode(5);

  REQUIRE(sink.received == std::vector{1, 2, 3});

  unit.encode(0);

  REQUIRE(sink.received == std::vector{1, 2, 3, 0, 4, 5});
  REQUIRE(sink.flushCount == 0);

  unit.encode(6);
  unit.flush();

  REQUIRE(sink.received == std::vector{1, 2, 3, 0, 4, 5, 0, 6});
  REQUIRE(sink.flushCount == 1);
}
