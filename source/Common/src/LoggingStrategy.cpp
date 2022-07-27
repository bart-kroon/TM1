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

#include <TMIV/Common/LoggingStrategyFmt.h>

#include <fmt/ostream.h>

#include <cassert>
#include <chrono>
#include <ostream>

using namespace std::string_view_literals;

namespace TMIV::Common {
auto operator<<(std::ostream &stream, LogLevel value) -> std::ostream & {
  switch (value) {
  case LogLevel::silent:
    return stream << "silent"sv;
  case LogLevel::error:
    return stream << "error"sv;
  case LogLevel::warning:
    return stream << "warning"sv;
  case LogLevel::info:
    return stream << "info"sv;
  case LogLevel::verbose:
    return stream << "verbose"sv;
  case LogLevel::debug:
    return stream << "debug"sv;
  default:
    return stream << static_cast<int32_t>(value);
  }
}

namespace {
struct LoggerSingleton {
  using Clock = std::chrono::steady_clock;

  [[nodiscard]] static auto instance() -> LoggerSingleton & {
    static LoggerSingleton singleton;
    return singleton;
  }

  Clock::time_point start = Clock::now();

  LoggingStrategy strategy = [this](LogLevel level, std::string_view what) {
    const std::chrono::duration<double> duration = Clock::now() - start;

    const auto prefix = [level]() {
      switch (level) {
      case LogLevel::error:
        return "ERROR: "sv;
      case LogLevel::warning:
        return "WARNING: "sv;
      case LogLevel::debug:
        return "DEBUG: "sv;
      default:
        return ""sv;
      }
    }();

    circumventLogger("{:013.6f}  {:7}  {}{}\n", duration.count(), level, prefix, what);
  };

  LogLevel maxLevel =
#ifdef NDEBUG
      LogLevel::info
#else
      LogLevel::debug
#endif
      ;
};
} // namespace

void changeMaxLogLevel(LogLevel value) { LoggerSingleton::instance().maxLevel = value; }

void replaceLoggingStrategy(LoggingStrategy value) {
  LoggerSingleton::instance().strategy = std::move(value);
}

void logMessage(LogLevel level, std::string_view what) noexcept {
  try {
    const auto &singleton = LoggerSingleton::instance();

    if (level <= singleton.maxLevel) {
      singleton.strategy(level, what);
    }
  } catch (...) {
    handleLogMessageException();
  }
}

void logError(std::string_view what) noexcept { logMessage(LogLevel::error, what); }

void logWarning(std::string_view what) noexcept { logMessage(LogLevel::warning, what); }

void logInfo(std::string_view what) noexcept { logMessage(LogLevel::info, what); }

void logVerbose(std::string_view what) noexcept { logMessage(LogLevel::verbose, what); }

void logDebug(std::string_view what) noexcept { logMessage(LogLevel::debug, what); }

[[noreturn]] void handleLogMessageException() noexcept {
  try {
    try {
      throw;
    } catch (std::exception &e) {
      circumventLogger("ERROR: Exception in TMIV::Common::logMessage: {}\n", e.what());
    } catch (...) {
      circumventLogger("ERROR: Unknown exception in TMIV::Common::logMessage.\n");
    }
  } catch (...) {
  }
  std::abort();
}
} // namespace TMIV::Common
