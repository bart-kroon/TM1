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

#ifndef TMIV_COMMON_TRAITS_H
#define TMIV_COMMON_TRAITS_H

#include <complex>
#include <type_traits>
#include <typeinfo>

namespace TMIV::Common {
template <typename T>
using NumericChecker =
    typename std::enable_if_t<std::is_arithmetic_v<T> || std::is_same_v<T, std::complex<float>> ||
                              std::is_same_v<T, std::complex<double>>>;

template <typename... Tn> struct same_type {};
template <typename T1, typename T2> struct same_type<T1, T2> {
  static constexpr bool value = std::is_same_v<T1, T2>;
};

template <typename T1, typename T2, typename... Tn> struct same_type<T1, T2, Tn...> {
  static constexpr bool value = std::is_same_v<T1, T2> && same_type<T1, Tn...>::value;
};

template <typename... Tn>
using SameTypeChecker = typename std::enable_if_t<same_type<Tn...>::value>;

template <typename T>
struct is_ordinal : public std::disjunction<std::is_integral<T>, std::is_enum<T>> {};

template <typename T> static constexpr bool is_ordinal_v = is_ordinal<T>::value;
} // namespace TMIV::Common

#endif
