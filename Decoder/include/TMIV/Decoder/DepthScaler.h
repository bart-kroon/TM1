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

#ifndef _TMIV_DECODER_DEPTHSCALER_H_
#define _TMIV_DECODER_DEPTHSCALER_H_

#include <algorithm>
#include <cassert>
#include <iostream>
#include <numeric>
#include <vector>

#include <TMIV/Common/Frame.h>
#include <TMIV/Common/Json.h>

namespace TMIV::Decoder {
using uchar = unsigned char;
using ushort = unsigned short;

using TMIV::Common::Vec2i;
using TMIV::Common::Vec3f;
using TMIV::Common::Vec3w;

template <class T> using Mat_ = TMIV::Common::heap::Matrix<T>;
using Mat1w = Mat_<uint16_t>;
using Mat3w = Mat_<Vec3w>;
using Mat1b = Mat_<uint8_t>;

class DepthMapAlignerColorBased {
public:
  DepthMapAlignerColorBased(int depthEdgeMagnitudeTh, float minForegroundConfidence);

  auto colorConfidence(const std::vector<ushort> &depthValues,
                       const std::vector<Vec3w> &colorValues,
                       const std::vector<uchar> &edgeMagnitudes) -> float;
  auto colorConfidenceAt(const Mat3w &yuv, const Mat1w &depth, const Mat1b &edgeMagnitudes,
                         const Vec2i &loc) -> float;
  auto operator()(const Mat3w &yuv, const Mat1w &depth, const Mat1b &edgeMagnitudes) -> Mat1w;

private:
  int m_depthEdgeMagnitudeTh = 11;
  float m_minForegroundConfidence = 0.4F;
  Mat1b m_markers;
  std::vector<Vec2i> m_kernelPoints;
  int m_B = 2;
};

class DepthMapAlignerCurvatureBased {
public:
  DepthMapAlignerCurvatureBased(int depthEdgeMagnitudeTh, int maxCurvature);

  auto curvature(const std::vector<ushort> &depthValues) -> int;
  auto curvatureAt(const Mat1w &depth, const Vec2i &loc) -> int;
  auto operator()(const Mat1w &depth, const Mat1b &edgeMagnitudes) -> Mat1w;

private:
  int m_depthEdgeMagnitudeTh = 11;
  int m_maxCurvature = 6;
  Mat1b m_markers;
  std::vector<Vec2i> kernelPoints;
  int m_B = 1;
};

class DepthUpscaler {
public:
  DepthUpscaler(int depthEdgeMagnitudeTh, float minForegroundConfidence, int maxCurvature);

  auto operator()(const Mat1w &depthD2, const Mat3w &yuv) -> Mat1w;
  auto operator()(const Mat1w &depthD2, const Mat1w &regionsD2, const Mat3w &yuv)
      -> std::pair<Mat1w, Mat1w>;

private:
  DepthMapAlignerColorBased m_alignerColor;
  DepthMapAlignerCurvatureBased m_alignerCurvature;
  Mat1b m_edgeMagnitudes1, m_edgeMagnitudes2;
  Mat1w m_depthUpscaled, m_depthColorAligned, m_depthCurvatureAligned;
  Mat1w m_regionsUpscaled, m_regionsColorAligned, m_regionsCurvatureAligned;
};

class DepthUpscalerAtlas {
public:
  DepthUpscalerAtlas(const Common::Json & /*rootNode*/, const Common::Json &componentNode);

  auto upsampleDepthAndOccupancyMapMVD(const TMIV::Common::MVD10Frame &atlas,
                                       const TMIV::Common::PatchIdMapList &maps) const
      -> std::pair<TMIV::Common::MVD10Frame, TMIV::Common::PatchIdMapList>;

private:
  int m_depthEdgeMagnitudeTh = 11;
  float m_minForegroundConfidence = 0.4F;
  int m_maxCurvature = 4;
};

} // namespace TMIV::Decoder

#endif