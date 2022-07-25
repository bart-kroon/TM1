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

#ifndef TMIV_COMMON_LOGGING_STRATEGY_H
#define TMIV_COMMON_LOGGING_STRATEGY_H

#include <array>
#include <functional>
#include <iosfwd>
#include <string_view>

namespace TMIV::Common {
// depending on the context:
//
//   * log level of individual log messages
//   * application verbosity setting
//
// LogLevel::silent supressed all log messages.
enum class LogLevel { silent, error, warning, info, verbose, debug };

// Enum to string, e.g. for CLI parsing
auto operator<<(std::ostream &stream, LogLevel value) -> std::ostream &;

// All available verbosity settings
static constexpr auto allLogLevels =
    std::array{LogLevel::silent, LogLevel::error,   LogLevel::warning,
               LogLevel::info,   LogLevel::verbose, LogLevel::debug};

// The signature of the function or closure (implementing a logging strategy) that is invoked when a
// log message is determined to have a log level that does not exceed the current verbosity setting
using LoggingStrategy = std::function<void(LogLevel, std::string_view)>;

// Provide a new callback function, e.g. when using this project as a library component
//
// The new value is required to be non-null.
void replaceLoggingStrategy(LoggingStrategy value);

// Change the verbosity setting. This operation does not provide a thread-safety guarantee
//
// The default setting is `info` for Release builds and `debug` for Debug builds.
void changeMaxLogLevel(LogLevel value);

// Provide a log message of specified log level.
//
// The message `what` is typically one but sometimes more lines and shall not be terminated by a new
// line character. The message shall not include formatting relating to the log level, e.g. "ERROR:"
// because this is the responsibility of the logging strategy.
//
// The log level shall be known and shall not be equal to `LogLevel::silent`.
//
// This function is `noexcept` to enable logging from destructors and `noexcept` functions.
// Exceptions are handled by `handleLogMessageException`.
void logMessage(LogLevel level, std::string_view what) noexcept;

// Equivalent to logMessage(LogLevel::error, what)
void logError(std::string_view what) noexcept;

// Equivalent to logMessage(LogLevel::warning, what)
void logWarning(std::string_view what) noexcept;

// Equivalent to logMessage(LogLevel::info, what)
void logInfo(std::string_view what) noexcept;

// Equivalent to logMessage(LogLevel::verbose, what)
void logVerbose(std::string_view what) noexcept;

// Equivalent to logMessage(LogLevel::debug, what)
void logDebug(std::string_view what) noexcept;

// Lippincott function to handle exceptions while logging. An attempt is made to print the exception
// message, and the program is terminated.
[[noreturn]] void handleLogMessageException() noexcept;
} // namespace TMIV::Common

#endif
