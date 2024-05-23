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

#ifndef TMIV_COMMON_LOGGING_STRATEGY_FMT_H
#define TMIV_COMMON_LOGGING_STRATEGY_FMT_H

#include <TMIV/Common/format.h>

#include "LoggingStrategy.h"

namespace TMIV::Common {
// Provide a log message of specified log level.
//
// This overload composes the message from a format string and arguments. The format string has
// `std::format` syntax which is similar to Python `str.format`:
//
//   * https://en.cppreference.com/w/cpp/utility/format/format
//   * https://fmt.dev/latest/index.html
//
// The message is typically one but sometimes more lines and shall not be terminated by a new line
// character. The message shall not include formatting relating to the log level, e.g. "ERROR:"
// because this is the responsibility of the logging strategy.
//
// The log level shall be known and shall not be equal to `LogLevel::silent`.
//
// This function is `noexcept` to enable logging from destructors and `noexcept` functions.
// Formatting errors are handled by printing the exception message using `LogLevel::error` level,
// and the function returns normally. All other exceptions are handled by
// `handleLogMessageException`.
template <typename... Args, typename = std::enable_if_t<0 != sizeof...(Args)>>
auto logMessage(LogLevel level, TMIV_FMT::format_string<Args...> fmt, Args &&...args) noexcept {
  try {
    logMessage(level, TMIV_FMT::format(fmt, std::forward<Args>(args)...));
  } catch (...) {
    handleLogMessageException();
  }
}

// Equivalent to logMessage(LogLevel::error, fmt, args...)
template <typename... Args, typename = std::enable_if_t<0 != sizeof...(Args)>>
void logError(TMIV_FMT::format_string<Args...> fmt, Args &&...args) noexcept {
  logMessage(LogLevel::error, fmt, std::forward<Args>(args)...);
}

// Equivalent to logMessage(LogLevel::warning, fmt, args...)
template <typename... Args, typename = std::enable_if_t<0 != sizeof...(Args)>>
void logWarning(TMIV_FMT::format_string<Args...> fmt, Args &&...args) noexcept {
  logMessage(LogLevel::warning, fmt, std::forward<Args>(args)...);
}

// Equivalent to logMessage(LogLevel::info, fmt, args...)
template <typename... Args, typename = std::enable_if_t<0 != sizeof...(Args)>>
void logInfo(TMIV_FMT::format_string<Args...> fmt, Args &&...args) noexcept {
  logMessage(LogLevel::info, fmt, std::forward<Args>(args)...);
}

// Equivalent to logMessage(LogLevel::verbose, fmt, args...)
template <typename... Args, typename = std::enable_if_t<0 != sizeof...(Args)>>
void logVerbose(TMIV_FMT::format_string<Args...> fmt, Args &&...args) noexcept {
  logMessage(LogLevel::verbose, fmt, std::forward<Args>(args)...);
}

// Equivalent to logMessage(LogLevel::debug, fmt, args...)
template <typename... Args, typename = std::enable_if_t<0 != sizeof...(Args)>>
void logDebug(TMIV_FMT::format_string<Args...> fmt, Args &&...args) noexcept {
  logMessage(LogLevel::debug, fmt, std::forward<Args>(args)...);
}

// Like std::print(fmt, args...), but document that it is desired to circumvent the logger
template <typename... Args>
void circumventLogger(TMIV_FMT::format_string<Args...> fmt, Args &&...args) {
  using TMIV_FMT::print; // Circumvent code quality check
  print(fmt, std::forward<Args>(args)...);
}
} // namespace TMIV::Common

#endif
