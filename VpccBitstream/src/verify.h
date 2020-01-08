/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

#ifndef _TMIV_VPCCBITSTREAM_VERIFY_H_
#define _TMIV_VPCCBITSTREAM_VERIFY_H_

#include <cstdlib>
#include <iostream>

// Checks against 23090-5 V-PCC specification
//
// These checks do not relate to 23090-12 profile restrictions.
#define VERIFY_VPCCBITSTREAM(condition)                                                            \
  static_cast<void>((!!(condition) ||                                                              \
                     (::TMIV::VpccBitstream::vpccError(#condition, __FILE__, __LINE__), false)))
#define VPCCBITSTREAM_ERROR(what) ::TMIV::VpccBitstream::vpccError(what, __FILE__, __LINE__)

// Check against (proposed) 23090-12 MIV specification
//
// As a rule of thumb checks while encoding/decoding V-PCC structures are performed only at the
// points where it saves implementation effort. For instance, by checking that a flag is false, it
// may save the implementation of a entire syntax structure. This "lazy" checking makes it easier to
// change the MIV specification/profile. MIV structures are checked similar to TMIV 3.0.
#define VERIFY_MIVBITSTREAM(condition)                                                             \
  static_cast<void>(                                                                               \
      (!!(condition) || (::TMIV::VpccBitstream::mivError(#condition, __FILE__, __LINE__), false)))
#define MIVBITSTREAM_ERROR(what) ::TMIV::VpccBitstream::mivError(what, __FILE__, __LINE__)

namespace TMIV::VpccBitstream {
[[noreturn]] void vpccError(char const *condition, char const *file, int line);
[[noreturn]] void mivError(char const *condition, char const *file, int line);
} // namespace TMIV::VpccBitstream

#endif