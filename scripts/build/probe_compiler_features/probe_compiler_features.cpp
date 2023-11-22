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

// https://en.cppreference.com/w/cpp/feature_test

#include <cctype>
#include <fstream>
#include <string_view>

#ifdef __has_include
#if __has_include(<version>)
#include <version>
#endif
#endif

[[nodiscard]] auto take_int(std::string_view text) {
  while (!text.empty() && !std::isdigit(text.back())) {
    text.remove_suffix(1);
  }
  if (text.empty()) {
    return std::string_view{"0"};
  }
  return text;
}

#define VALUE(name) #name
#define ENTRY(name) stream << ",\n    \"" << #name << "\": " << take_int(VALUE(name));

auto main(int argc, char *argv[]) -> int32_t {
  if (argc != 2) {
    return 1;
  }
  std::ofstream stream(argv[1]);
  stream << "{\n";
  stream << "    \"cxx_standard\": " << cxx_standard;
  ENTRY(__cpp_lib_format);
  ENTRY(__cpp_lib_print);
  ENTRY(__cpp_lib_stacktrace);
  ENTRY(__cpp_core_TODO);
  stream << "\n}\n";
  return 0;
}
