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

#ifndef _TMIV_MIVBITSTREAM_VERIFY_H_
#define _TMIV_MIVBITSTREAM_VERIFY_H_

#include <cstdlib>
#include <iostream>

// Checks against (draft) ISO/IEC 23090-5 V3C and V-PCC specification
//
// These checks do not relate to ISO/IEC 23090-12 extensions or restrictions.
#define VERIFY_V3CBITSTREAM(condition)                                                             \
  static_cast<void>(                                                                               \
      (!!(condition) || (::TMIV::MivBitstream::v3cError(#condition, __FILE__, __LINE__), false)))
#define V3CBITSTREAM_ERROR(what) ::TMIV::MivBitstream::v3cError(what, __FILE__, __LINE__)

// Check against (draft) ISO/IEC 23090-12 MIV specification
//
// These checks relate to ISO/IEC 23090-12 extensions of or restrictions on ISO/IEC 23090-5.
#define VERIFY_MIVBITSTREAM(condition)                                                             \
  static_cast<void>(                                                                               \
      (!!(condition) || (::TMIV::MivBitstream::mivError(#condition, __FILE__, __LINE__), false)))
#define MIVBITSTREAM_ERROR(what) ::TMIV::MivBitstream::mivError(what, __FILE__, __LINE__)

// Known limitation of the current implementation (not in line with ISO/IEC 23090-12)
#define LIMITATION(condition)                                                                      \
  static_cast<void>(                                                                               \
      (!!(condition) ||                                                                            \
       (::TMIV::MivBitstream::notImplemented(#condition, __FILE__, __LINE__), false)))
#define NOT_IMPLEMENTED(what) ::TMIV::MivBitstream::notImplemented(what, __FILE__, __LINE__)

namespace TMIV::MivBitstream {
[[noreturn]] void v3cError(char const *condition, char const *file, int line);
[[noreturn]] void mivError(char const *condition, char const *file, int line);
[[noreturn]] void notImplemented(char const *condition, char const *file, int line);
} // namespace TMIV::MivBitstream

#endif