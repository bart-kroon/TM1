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

#ifndef TMIV_MIVBITSTREAM_VIEWPARAMSLIST_H
#define TMIV_MIVBITSTREAM_VIEWPARAMSLIST_H

#include "CafMivExtension.h"

#include <TMIV/Common/Json.h>
#include <TMIV/Common/Vector.h>

namespace TMIV::MivBitstream {
struct Pose {
  Common::Vec3f position;
  Common::QuatD orientation{Common::neutralOrientationD};

  auto printTo(std::ostream &stream, uint16_t viewIdx) const -> std::ostream &;

  [[nodiscard]] auto operator==(const Pose &other) const -> bool;
  [[nodiscard]] auto operator!=(const Pose &other) const -> bool { return !operator==(other); }

  [[nodiscard]] static auto decodeFrom(const CameraExtrinsics &ce) -> Pose;
  [[nodiscard]] auto encodeToCameraExtrinsics() const -> CameraExtrinsics;

  [[nodiscard]] auto hasEqualCodeTo(const Pose &other) const {
    return encodeToCameraExtrinsics() == other.encodeToCameraExtrinsics();
  }
};

struct ViewParams {
  ViewId viewId;
  CameraIntrinsics ci;
  Pose pose;
  DepthQuantization dq;
  ChromaScaling cs;
  std::optional<PruningParents> pp;

  ViewParams() = default;
  ViewParams(const CameraIntrinsics &ci_, const Pose &pose_, DepthQuantization dq_,
             const ChromaScaling &cs_, std::optional<PruningParents> pp_, std::string name_)
      : ci{ci_}
      , pose{pose_}
      , dq{std::move(dq_)}
      , cs{cs_}
      , pp{std::move(pp_)}
      , name{std::move(name_)} {}

  // Not in the specification. Just to improve screen output
  std::string name{};

  // Not part of the bitstream. Does the depth map have invalid/non-occupied?
  bool hasOccupancy{};

  // Is this a basic view or an additional view?
  bool isBasicView{true};

  // Is this view a regular view (e.g. captured by a camera) or the result of an inpainting process?
  bool viewInpaintFlag{};

  // Number of layers in MPI. Not in the specification, but needed to handle MPI.
  int32_t nbMpiLayers{1};

  auto printTo(std::ostream &stream, uint16_t viewIdx) const -> std::ostream &;
  auto operator==(const ViewParams &other) const -> bool;
  auto operator!=(const ViewParams &other) const -> bool { return !operator==(other); }

  // Load a single (source) camera from a JSON metadata file (RVS 3.x format)
  //
  // The parameter is a an item of the viewParamsList node (a JSON object).
  explicit ViewParams(const Common::Json &node);

  explicit operator Common::Json() const;

  [[nodiscard]] auto viewRoot() const -> bool;
  [[nodiscard]] auto viewNumParents() const -> uint16_t;
  [[nodiscard]] auto viewParentIdx(uint16_t i) const -> uint16_t;
};

// Vector of ViewParams with indexing of view ID
class ViewParamsList : public std::vector<ViewParams> {
public:
  void constructViewIdIndex();
  void assignViewIds(std::vector<uint16_t> sourceCameraIDs);
  [[nodiscard]] auto maxViewIdValue() const noexcept -> uint16_t;

  [[nodiscard]] auto indexOf(ViewId viewId) const {
    VERIFY(viewId.m_value < m_viewIdIndex.size());
    const auto viewIdx = m_viewIdIndex[viewId.m_value];
    VERIFY(viewIdx < size());
    VERIFY(operator[](viewIdx).viewId == viewId);
    return viewIdx;
  }

  using std::vector<ViewParams>::operator[];

  auto operator[](ViewId viewId) const -> decltype(auto) { return operator[](indexOf(viewId)); }
  auto operator[](ViewId viewId) -> decltype(auto) { return operator[](indexOf(viewId)); }

private:
  std::vector<uint16_t> m_viewIdIndex;
};
} // namespace TMIV::MivBitstream

#endif
