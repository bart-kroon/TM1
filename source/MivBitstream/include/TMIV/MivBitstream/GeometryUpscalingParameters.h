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

#ifndef TMIV_MIVBITSTREAM_GEOMETRYUPSCALINGPARAMETERS_H
#define TMIV_MIVBITSTREAM_GEOMETRYUPSCALINGPARAMETERS_H

#include <TMIV/Common/Bitstream.h>

#include <vector>

namespace TMIV::MivBitstream {
// 23090-12: Geometry upscaling parameter types
enum class GupType : uint8_t {
  HVR // Hypothetical view renderer
};

auto operator<<(std::ostream & /*stream*/, GupType /*x*/) -> std::ostream &;

// 23090-12: geometry_upscaling_parameters( payloadSize )
class GeometryUpscalingParameters {
public:
  [[nodiscard]] auto gup_type() const noexcept -> GupType;
  [[nodiscard]] auto gup_erode_threshold() const -> Common::Half;
  [[nodiscard]] auto gup_delta_threshold() const noexcept -> uint32_t;
  [[nodiscard]] auto gup_max_curvature() const noexcept -> uint8_t;

  auto gup_type(GupType value) noexcept -> GeometryUpscalingParameters &;
  auto gup_erode_threshold(Common::Half value) noexcept -> GeometryUpscalingParameters &;
  auto gup_delta_threshold(uint32_t value) noexcept -> GeometryUpscalingParameters &;
  auto gup_max_curvature(uint8_t value) noexcept -> GeometryUpscalingParameters &;

  friend auto operator<<(std::ostream &stream, const GeometryUpscalingParameters &x)
      -> std::ostream &;

  auto operator==(const GeometryUpscalingParameters &other) const noexcept -> bool;
  auto operator!=(const GeometryUpscalingParameters &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> GeometryUpscalingParameters;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  GupType m_gup_type{};
  Common::Half m_gup_erode_threshold{};
  uint32_t m_gup_delta_threshold{};
  uint8_t m_gup_max_curvature{};
};
} // namespace TMIV::MivBitstream

#endif
