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

#include <catch2/catch_test_macros.hpp>

#include <TMIV/Common/LoggingStrategy.h>

#include <sstream>

using TMIV::Common::allLogLevels;
using TMIV::Common::changeMaxLogLevel;
using TMIV::Common::logDebug;
using TMIV::Common::logError;
using TMIV::Common::LoggingStrategy;
using TMIV::Common::logInfo;
using TMIV::Common::LogLevel;
using TMIV::Common::logVerbose;
using TMIV::Common::logWarning;
using TMIV::Common::replaceLoggingStrategy;

using namespace std::string_view_literals;

namespace test {
struct LogEntry {
  LogEntry(LogLevel level, std::string what) : level{level}, what{std::move(what)} {}

  LogLevel level;
  std::string what;
};
using Log = std::vector<LogEntry>;

auto fakeLoggingStrategy(Log &log) {
  return
      [&log](LogLevel level, std::string_view what) { log.emplace_back(level, std::string{what}); };
}
} // namespace test

TEST_CASE("TMIV::Common::LogLevel") {
  static_assert(static_cast<int32_t>(LogLevel::silent) == 0);
  static_assert(static_cast<int32_t>(LogLevel::error) == 1);
  static_assert(static_cast<int32_t>(LogLevel::warning) == 2);
  static_assert(static_cast<int32_t>(LogLevel::info) == 3);
  static_assert(static_cast<int32_t>(LogLevel::verbose) == 4);
  static_assert(static_cast<int32_t>(LogLevel::debug) == 5);
}

TEST_CASE("TMIV::Common::allLogLevels") {
  static_assert(allLogLevels[0] == LogLevel::silent);
  static_assert(allLogLevels[1] == LogLevel::error);
  static_assert(allLogLevels[2] == LogLevel::warning);
  static_assert(allLogLevels[3] == LogLevel::info);
  static_assert(allLogLevels[4] == LogLevel::verbose);
  static_assert(allLogLevels[5] == LogLevel::debug);
}

TEST_CASE("operator << (std::ostream&, TMIV::Common::LogLevel") {
  std::ostringstream buffer;
  buffer << '\n';

  for (const auto logLevel : allLogLevels) {
    buffer << static_cast<int32_t>(logLevel) << " " << logLevel << '\n';
  }

  for (const auto logLevel : {static_cast<LogLevel>(100), static_cast<LogLevel>(-3)}) {
    buffer << static_cast<int32_t>(logLevel) << " " << logLevel << '\n';
  }

  CHECK(buffer.str() == R"(
0 silent
1 error
2 warning
3 info
4 verbose
5 debug
100 100
-3 -3
)");
}

TEST_CASE("Default logging strategy") {
  for (const auto level : allLogLevels) {
    if (level != LogLevel::silent) {
      logMessage(level, "Test default logging strategy (by visual inspection)"sv);
    }
  }
}

TEST_CASE("TMIV::Common::replaceLoggingStrategy") {
  test::Log log;
  LoggingStrategy strategy = fakeLoggingStrategy(log);

  replaceLoggingStrategy(strategy);

  logError("Banana"sv);
  CHECK(log.size() == 1);
  CHECK(log.back().what == "Banana"sv);

  replaceLoggingStrategy(strategy);

  logInfo("Citrus"sv);
  CHECK(log.size() == 2);
  CHECK(log.back().what == "Citrus"sv);

  replaceLoggingStrategy([](auto...) {});

  logInfo("Date"sv);
  CHECK(log.size() == 2);
}

TEST_CASE("TMIV::Common::logMessage") {
  test::Log log;
  replaceLoggingStrategy(fakeLoggingStrategy(log));
  changeMaxLogLevel(LogLevel::debug);

  logError(""sv);
  CHECK(log.back().level == LogLevel::error);

  logWarning(""sv);
  CHECK(log.back().level == LogLevel::warning);

  logInfo(""sv);
  CHECK(log.back().level == LogLevel::info);

  logVerbose(""sv);
  CHECK(log.back().level == LogLevel::verbose);

  logDebug(""sv);
  CHECK(log.back().level == LogLevel::debug);
}

TEST_CASE("TMIV::Common::changeMaxLogLevel") {
  const auto probe = [](LogLevel logLevel) {
    test::Log log;
    replaceLoggingStrategy(fakeLoggingStrategy(log));
    changeMaxLogLevel(logLevel);

    logError(""sv);
    logWarning(""sv);
    logInfo(""sv);
    logVerbose(""sv);
    logDebug(""sv);

    auto result = std::vector<LogLevel>{};

    for (const auto &entry : log) {
      result.push_back(entry.level);
    }

    return result;
  };

  CHECK(probe(LogLevel::debug) == std::vector{LogLevel::error, LogLevel::warning, LogLevel::info,
                                              LogLevel::verbose, LogLevel::debug});
  CHECK(probe(LogLevel::verbose) ==
        std::vector{LogLevel::error, LogLevel::warning, LogLevel::info, LogLevel::verbose});
  CHECK(probe(LogLevel::info) == std::vector{LogLevel::error, LogLevel::warning, LogLevel::info});
  CHECK(probe(LogLevel::warning) == std::vector{LogLevel::error, LogLevel::warning});
  CHECK(probe(LogLevel::error) == std::vector{LogLevel::error});
  CHECK(probe(LogLevel::silent).empty());
}
