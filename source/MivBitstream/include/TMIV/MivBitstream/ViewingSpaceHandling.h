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

#ifndef TMIV_MIVBITSTREAM_VIEWINGSPACEHANDLING_H
#define TMIV_MIVBITSTREAM_VIEWINGSPACEHANDLING_H

#include <TMIV/Common/Bitstream.h>

#include <vector>

namespace TMIV::MivBitstream {
// 23090-12: Viewing space handling device classes
enum class VhDeviceClass : uint8_t { VHDC_ALL, VHDC_HMD, VHDC_PHONE, VHDC_ASD };

// 23090-12: Viewing space handling application classes
enum class VhApplicationClass : uint8_t { VHAC_ALL, VHAC_AR, VHAC_VR, VHAC_WEB, VHAC_SD };

// 23090-12: Viewing space handling method
enum class VhMethod : uint8_t {
  VHM_NULL,
  VHM_RENDER,
  VHM_FADE,
  VHM_EXTRAP,
  VHM_RESET,
  VHM_STRETCH,
  VHM_ROTATE
};

auto operator<<(std::ostream & /*stream*/, VhDeviceClass /*x*/) -> std::ostream &;
auto operator<<(std::ostream & /*stream*/, VhApplicationClass /*x*/) -> std::ostream &;
auto operator<<(std::ostream & /*stream*/, VhMethod /*x*/) -> std::ostream &;

struct HandlingOption {
  VhDeviceClass vs_handling_device_class;
  VhApplicationClass vs_handling_application_class;
  VhMethod vs_handling_method;

  auto operator==(const HandlingOption &other) const noexcept -> bool;
  auto operator!=(const HandlingOption &other) const noexcept -> bool;
};
using HandlingOptionList = std::vector<HandlingOption>;

// 23090-12: viewing_space_handling()
class ViewingSpaceHandling {
public:
  ViewingSpaceHandling() = default;
  explicit ViewingSpaceHandling(HandlingOptionList /*value*/);

  [[nodiscard]] auto vs_handling_options_count() const noexcept -> size_t;
  [[nodiscard]] auto vs_handling_device_class(size_t h) const -> VhDeviceClass;
  [[nodiscard]] auto vs_handling_application_class(size_t h) const -> VhApplicationClass;
  [[nodiscard]] auto vs_handling_method(size_t h) const -> VhMethod;

  friend auto operator<<(std::ostream &stream, const ViewingSpaceHandling &x) -> std::ostream &;

  auto operator==(const ViewingSpaceHandling &other) const noexcept -> bool;
  auto operator!=(const ViewingSpaceHandling &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> ViewingSpaceHandling;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  HandlingOptionList m_handlingOptionList;
};
} // namespace TMIV::MivBitstream

#endif
