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

#ifndef TMIV_COMMON_VERIFY_H
#define TMIV_COMMON_VERIFY_H

#include "LoggingStrategy.h"

#include <cstdlib>
#include <limits>
#include <sstream>
#include <stdexcept>

#if defined(__clang__) || defined(__GNUC__)
#define LIKELY(x) __builtin_expect(static_cast<bool>(x), 1)
#else
#define LIKELY(x) (static_cast<bool>(x))
#endif

// Check that externally provided information (e.g. files, parameters, etc.) is correct
//
//  * This is an external error source and thus an exception of type std::runtime_error will be
//    thrown.
//  * When it is not clear from the context of a function if the cause is external or internal, then
//    it is better to assume that the cause is external. (When in doubt use VERIFY over
//    PRECONDITION.)
#define VERIFY(condition)                                                                          \
  static_cast<void>(LIKELY(condition) ||                                                           \
                    (::TMIV::Common::runtimeError(#condition, __FILE__, __LINE__), false))

// Check bitstream against (draft) ISO/IEC 23090-5 V3C and V-PCC specification
//
//  * The bitstream is an external error source and thus an exception of type
//    TMIV::Common::V3cBitstreamError will be thrown.
//  * When it is not clear from the context of a function if the cause is external or internal, then
//    it is better to assume that the cause is external. (When in doubt use VERIFY_V3CBITSTREAM over
//    PRECONDITION.)
#define VERIFY_V3CBITSTREAM(condition)                                                             \
  static_cast<void>(LIKELY(condition) ||                                                           \
                    (::TMIV::Common::v3cBitstreamError(#condition, __FILE__, __LINE__), false))

#define V3CBITSTREAM_ERROR(what) ::TMIV::Common::v3cBitstreamError(what, __FILE__, __LINE__)

// Check bitstream against (draft) ISO/IEC 23090-12 MIV specification
//
//  * The bitstream is an external error source and thus an exception of type
//    TMIV::Common::MivBitstreamError will be thrown.
//  * When it is not clear from the context of a function if the cause is external or internal, then
//    it is better to assume that the cause is external. (When in doubt use VERIFY_MIVBITSTREAM over
//    PRECONDITION.)
#define VERIFY_MIVBITSTREAM(condition)                                                             \
  static_cast<void>(LIKELY(condition) ||                                                           \
                    (::TMIV::Common::mivBitstreamError(#condition, __FILE__, __LINE__), false))

#define MIVBITSTREAM_ERROR(what) ::TMIV::Common::mivBitstreamError(what, __FILE__, __LINE__)

// Check against general bitstream conformance
//
//  * The bitstream is an external error source and thus an exception of type
//    TMIV::Common::BitstreamError will be thrown.
//  * When it is not clear from the context of a function if the cause is external or internal, then
//    it is better to assume that the cause is external. (When in doubt use VERIFY_BITSTREAM over
//    PRECONDITION.)
#define VERIFY_BITSTREAM(condition)                                                                \
  static_cast<void>(LIKELY(condition) ||                                                           \
                    (::TMIV::Common::bitstreamError(#condition, __FILE__, __LINE__), false))

#define BITSTREAM_ERROR(what) ::TMIV::Common::bitstreamError(what, __FILE__, __LINE__)

// Known limitation of the current implementation (not in line with ISO/IEC 23090-12)
//
// * When this triggers, this is always a bug in the test model.
// * This is an internal error source and thus an abnormal program termination will be triggered.
#define LIMITATION(condition)                                                                      \
  static_cast<void>((LIKELY(condition) ||                                                          \
                     (::TMIV::Common::assertionFailed(#condition, __FILE__, __LINE__), false)))

// Check an assumption. Unlike the assert macro from <cassert>, which shall not be used in this
// project, assumptions are checked regardless of the build type. This is a test model and
// correctness is more important than efficiency.
#define ASSERT(condition)                                                                          \
  static_cast<void>(LIKELY(condition) ||                                                           \
                    (::TMIV::Common::assertionFailed(#condition, __FILE__, __LINE__), false))

#define RUNTIME_ERROR(what) ::TMIV::Common::runtimeError(what, __FILE__, __LINE__)

// Check for a precondition on an operation that will start (assumptions on the input)
//
//  * When this triggers, this is always a bug in the test model.
//  * This is an internal error source and thus an abnormal program termination will be triggered.
#define PRECONDITION(condition)                                                                    \
  static_cast<void>((LIKELY(condition) ||                                                          \
                     (::TMIV::Common::assertionFailed(#condition, __FILE__, __LINE__), false)))

// Check for a postcondition on an operation that just took place (assumptions on the output)
//
//  * When this triggers, this is always a bug in the test model.
//  * This is an internal error source and thus an abnormal program termination will be triggered.
#define POSTCONDITION(condition)                                                                   \
  static_cast<void>((LIKELY(condition) ||                                                          \
                     (::TMIV::Common::assertionFailed(#condition, __FILE__, __LINE__), false)))

// Mark unreachable code
//
//  * When this triggers, this is always a bug in the test model.
//  * This is an internal error source and thus an abnormal program termination will be triggered.
#define UNREACHABLE                                                                                \
  ::TMIV::Common::assertionFailed("This code was assumed to be unreachable", __FILE__, __LINE__)

// Mark stubbed code
//
//  * When this triggers, this is always a bug in the test model.
//  * This is an internal error source and thus an abnormal program termination will be triggered.
#define NOT_IMPLEMENTED ::TMIV::Common::assertionFailed("Not implemented", __FILE__, __LINE__)

namespace TMIV::Common {
// Report that a bitstream does not conform to the relevant (draft) specification
//
//  * When it is not clear from the context of a function if the cause is external or internal, then
//    it is better to assume that the cause is external, and throw an exception.
class BitstreamError : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

// Report that a bitstream does not conform to the (draft) ISO/IEC 23090-5 V3C specification
//
//  * When it is not clear from the context of a function if the cause is external or internal, then
//    it is better to assume that the cause is external, and throw an exception.
class V3cBitstreamError : public BitstreamError {
  using BitstreamError::BitstreamError;
};

// Report that a bitstream does not conform to the (draft) ISO/IEC 23090-12 MIV specification
//
//  * When it is not clear from the context of a function if the cause is external or internal, then
//    it is better to assume that the cause is external, and throw an exception.
class MivBitstreamError : public BitstreamError {
  using BitstreamError::BitstreamError;
};

// Report that the test model does not support the bitstream based on provided profile-tier-level
// (PTL) information
//
//  * When it is not clear from the context of a function if the cause is external or internal, then
//    it is better to assume that the cause is external, and throw an exception.
class PtlError : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

// NOTE(BK): All inline to let the clang-tidy bugprone-exception-escape observe the throws

inline auto message(char const *introduction, char const *condition, char const *file, int32_t line)
    -> std::string {
  std::ostringstream stream;
  stream << introduction << ":\n\t" << condition << "\n\t[" << file << "@" << line << "]\n";
  return stream.str();
}

[[noreturn]] inline void runtimeError(char const *condition, char const *file, int32_t line) {
  throw std::runtime_error{message("Runtime error", condition, file, line)};
}

[[noreturn]] inline void v3cBitstreamError(char const *condition, char const *file, int32_t line) {
  throw V3cBitstreamError{
      message("Failed to parse/decode ISO/IEC 23090-5 V3C bitstream", condition, file, line)};
}

[[noreturn]] inline void mivBitstreamError(char const *condition, char const *file, int32_t line) {
  throw MivBitstreamError{
      message("Failed to parse/decode ISO/IEC 23090-12 MIV bitstream", condition, file, line)};
}

[[noreturn]] inline void bitstreamError(char const *condition, char const *file, int32_t line) {
  throw BitstreamError{message("Failed to parse/decode bitstream", condition, file, line)};
}

void logStacktrace();

[[noreturn]] inline void assertionFailed(char const *condition, char const *file,
                                         int32_t line) noexcept {
  try {
    logStacktrace();
    logError(message("Assertion failed", condition, file, line));
  } catch (...) {
  }
  std::abort();
}

// Down cast with bounds preconditions (internal error cause)
//
// The name suggests typical use but the function can also be used as the identity function or for
// upcasting. (When types are not known in a generic context.) Any non-applicable tests are removed
// at compile time.
template <typename Out, typename In> constexpr auto downCast(In input) noexcept -> Out {
  // NOTE(BK): There is near code duplication between downCast, verifyDownCast and assertDownCast,
  //           but the only C++17 alternative is to use a macro. Whenever changing this
  //           implementation, make the same change to the other two.

  static_assert(std::is_arithmetic_v<In> && std::is_integral_v<Out>);

  if constexpr (std::is_signed_v<In> && std::is_unsigned_v<Out>) {
    PRECONDITION(0 <= input);
  }
  if constexpr (std::numeric_limits<In>::digits > std::numeric_limits<Out>::digits) {
    if constexpr (std::is_signed_v<In> && std::is_signed_v<Out>) {
      PRECONDITION(std::numeric_limits<Out>::min() <= input);
    }
    PRECONDITION(input <= static_cast<In>(std::numeric_limits<Out>::max()));
  }
  return static_cast<Out>(input);
}

// Down cast with bounds verification (potentially external error cause, may throw)
//
// The name suggests typical use but the function can also be used as the identity function or for
// upcasting. (When types are not known in a generic context.) Any non-applicable tests are removed
// at compile time.
template <typename Out, typename In> constexpr auto verifyDownCast(In input) -> Out {
  // NOTE(BK): There is near code duplication between downCast, verifyDownCast and assertDownCast,
  //           but the only C++17 alternative is to use a macro. Whenever changing this
  //           implementation, make the same change to the other two.

  static_assert(std::is_arithmetic_v<In> && std::is_integral_v<Out>);

  if constexpr (std::is_signed_v<In> && std::is_unsigned_v<Out>) {
    VERIFY(0 <= input);
  }
  if constexpr (std::numeric_limits<In>::digits > std::numeric_limits<Out>::digits) {
    if constexpr (std::is_signed_v<In> && std::is_signed_v<Out>) {
      VERIFY(std::numeric_limits<Out>::min() <= input);
    }
    VERIFY(input <= static_cast<In>(std::numeric_limits<Out>::max()));
  }
  return static_cast<Out>(input);
}

// Down cast with bounds assertion (internal error cause, hot-loop)
//
// The name suggests typical use but the function can also be used as the identity function or for
// upcasting. (When types are not known in a generic context.) Any non-applicable tests are removed
// at compile time.
template <typename Out, typename In> constexpr auto assertDownCast(In input) noexcept -> Out {
  // NOTE(BK): There is near code duplication between downCast, verifyDownCast and assertDownCast,
  //           but the only C++17 alternative is to use a macro. Whenever changing this
  //           implementation, make the same change to the other two.

  static_assert(std::is_arithmetic_v<In> && std::is_integral_v<Out>);

  if constexpr (std::is_signed_v<In> && std::is_unsigned_v<Out>) {
    ASSERT(0 <= input);
  }
  if constexpr (std::numeric_limits<In>::digits > std::numeric_limits<Out>::digits) {
    if constexpr (std::is_signed_v<In> && std::is_signed_v<Out>) {
      ASSERT(std::numeric_limits<Out>::min() <= input);
    }
    ASSERT(input <= static_cast<In>(std::numeric_limits<Out>::max()));
  }
  return static_cast<Out>(input);
}

// Like gsl::at but prints a message before abnormal program termination and supports nested arrays
template <typename Container>
constexpr auto at(Container &container, size_t index) noexcept -> decltype(auto) {
  using std::size;
  PRECONDITION(index < size(container));
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
  return container[index];
}

// Like gsl::at but prints a message before abnormal program termination and supports nested arrays
template <typename T>
constexpr auto at(const std::initializer_list<T> init, size_t index) noexcept -> decltype(auto) {
  PRECONDITION(index < init.size());
  return *(init.begin() + index);
}

// Like gsl::at but prints a message before abnormal program termination and supports nested arrays
template <typename Container, typename... SizeT>
constexpr auto at(Container &container, size_t index0, SizeT... index) noexcept -> decltype(auto) {
  return at(at(container, index0), index...);
}

[[nodiscard]] auto handleException() noexcept -> int32_t;
} // namespace TMIV::Common

#endif
