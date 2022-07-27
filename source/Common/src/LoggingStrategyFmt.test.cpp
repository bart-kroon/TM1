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

#include <catch2/catch.hpp>

#include <TMIV/Common/LoggingStrategyFmt.h>

using TMIV::Common::allLogLevels;
using TMIV::Common::changeMaxLogLevel;
using TMIV::Common::circumventLogger;
using TMIV::Common::logDebug;
using TMIV::Common::logError;
using TMIV::Common::logInfo;
using TMIV::Common::LogLevel;
using TMIV::Common::logVerbose;
using TMIV::Common::logWarning;
using TMIV::Common::replaceLoggingStrategy;

using namespace std::string_view_literals;

TEST_CASE("TMIV::Common::logError(fmt, args...)") {
  auto callCount = 0;

  changeMaxLogLevel(LogLevel::debug);
  replaceLoggingStrategy([&callCount](LogLevel level, std::string_view what) {
    ++callCount;
    CHECK(level == LogLevel::error);
    CHECK(what == "100 x 200"sv);
  });

  logError("{1} x {0}", 200, 100);
  CHECK(callCount == 1);
}

TEST_CASE("Format errors are handled by printing and returning normally") {
  auto callCount = 0;
  static constexpr auto before = "Before causing a format error"sv;
  static constexpr auto after = "After causing a format error"sv;

  changeMaxLogLevel(LogLevel::debug);
  replaceLoggingStrategy([&callCount](LogLevel level, std::string_view what) {
    ++callCount;
    CAPTURE(callCount, level, what);

    switch (callCount) {
    case 1:
      CHECK(level == LogLevel::info);
      CHECK(what == before);
      break;
    case 2:
      CHECK(level == LogLevel::error);
      circumventLogger("{}\n", what);
      break;
    case 3:
      CHECK(level == LogLevel::info);
      CHECK(what == after);
      break;
    default:
      FAIL("Too many calls");
    }
  });

  logInfo(before);
  logMessage(LogLevel::warning, fmt::runtime("Three arguments {} {} {}"), 1, 2);
  logInfo(after);
  CHECK(callCount == 3);
}

TEST_CASE("TMIV::Common::logWarning(fmt, args...)") {
  auto callCount = 0;

  changeMaxLogLevel(LogLevel::debug);
  replaceLoggingStrategy([&callCount](LogLevel level, std::string_view what) {
    ++callCount;
    CHECK(level == LogLevel::warning);
    CHECK(what == "1920 x 1080"sv);
  });

  logWarning("{} x {}", 1920, 1080);
  CHECK(callCount == 1);
}

TEST_CASE("TMIV::Common::logInfo(fmt, args...)") {
  auto callCount = 0;

  changeMaxLogLevel(LogLevel::debug);
  replaceLoggingStrategy([&callCount](LogLevel level, std::string_view what) {
    ++callCount;
    CHECK(level == LogLevel::info);
    CHECK(what == "3 x 4"sv);
  });

  logInfo("{} x {}", 3, 4);
  CHECK(callCount == 1);
}

TEST_CASE("TMIV::Common::logVerbose(fmt, args...)") {
  auto callCount = 0;

  changeMaxLogLevel(LogLevel::debug);
  replaceLoggingStrategy([&callCount](LogLevel level, std::string_view what) {
    ++callCount;
    CHECK(level == LogLevel::verbose);
    CHECK(what == "1920 x 1080"sv);
  });

  logVerbose("{} x {}"sv, 1920, 1080);
  CHECK(callCount == 1);
}

TEST_CASE("TMIV::Common::logDebug(fmt, args...)") {
  auto callCount = 0;

  changeMaxLogLevel(LogLevel::debug);
  replaceLoggingStrategy([&callCount](LogLevel level, std::string_view what) {
    ++callCount;
    CHECK(level == LogLevel::debug);
    CHECK(what == "2048 x 1080"sv);
  });

  logDebug("{} x {}"sv, 2048, 1080);
  CHECK(callCount == 1);
}
