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

#include <catch2/catch_test_macros.hpp>

#include "PiecewiseLinearDepthScaling.h"

using namespace std::string_view_literals;

TEST_CASE("edge detection for piecwise linear scaling") {
  std::vector<std::vector<int32_t>> geometryUnit(3, std::vector(3, 0));
  geometryUnit[0] = {0, 10, 100};
  geometryUnit[1] = {0, 10, 100};
  geometryUnit[2] = {0, 10, 100};

  CHECK(TMIV::Encoder::plsEdgeDetection(geometryUnit, 40));
}

TEST_CASE("histogram normalization in PLS, for CG seq") {
  const auto piece_num = 4;

  std::vector<int32_t> histEdge;
  histEdge.assign(piece_num, 0);
  histEdge[0] = 300;
  histEdge[1] = 200;
  histEdge[2] = 500;
  histEdge[3] = 1000;

  bool lowDepthQuality = false;
  int32_t minDepthVal = 1000;
  int32_t maxDepthVal = 5000;

  std::vector<double> mapped_pivot;
  mapped_pivot = TMIV::Encoder::plsNormalizeHistogram(histEdge, piece_num, lowDepthQuality,
                                                      minDepthVal, maxDepthVal);

  CHECK(static_cast<int32_t>(mapped_pivot[0]) == 1000);
  CHECK(static_cast<int32_t>(mapped_pivot[1]) == 1866);
  CHECK(static_cast<int32_t>(mapped_pivot[2]) == 2666);
  CHECK(static_cast<int32_t>(mapped_pivot[3]) == 3666);
  CHECK(static_cast<int32_t>(mapped_pivot[4]) == 5000);
}

TEST_CASE("histogram normalization in PLS, for NC sequences") {
  const auto piece_num = 4;

  std::vector<int32_t> histEdge;
  histEdge.assign(piece_num, 0);
  histEdge[0] = 300;
  histEdge[1] = 200;
  histEdge[2] = 500;
  histEdge[3] = 1000;

  bool lowDepthQuality = true;
  int32_t minDepthVal = 1000;
  int32_t maxDepthVal = 5000;

  std::vector<double> mapped_pivot;
  mapped_pivot = TMIV::Encoder::plsNormalizeHistogram(histEdge, piece_num, lowDepthQuality,
                                                      minDepthVal, maxDepthVal);

  CHECK(static_cast<int32_t>(mapped_pivot[0]) == 1000);
  CHECK(static_cast<int32_t>(mapped_pivot[1]) == 1562);
  CHECK(static_cast<int32_t>(mapped_pivot[2]) == 2062);
  CHECK(static_cast<int32_t>(mapped_pivot[3]) == 2750);
  CHECK(static_cast<int32_t>(mapped_pivot[4]) == 3750);
}

TEST_CASE("calculation remapped geometry value in PLS, for CG sequences") {
  const auto piece_num = 4;

  int32_t minDepthVal = 1000;
  int32_t maxDepthVal = 5000;
  uint16_t geometry = 2000;
  std::vector<double> mapped_pivot = {1000, 1500, 4000, 4500, 5000};
  bool lowDepthQuality = false;

  geometry = TMIV::Encoder::plsDepthMapping(minDepthVal, maxDepthVal, piece_num, geometry,
                                            mapped_pivot, lowDepthQuality);

  CHECK(geometry == 8200);
}

TEST_CASE("calculation remapped geometry value in PLS, for NC sequences") {
  const auto piece_num = 4;

  int32_t minDepthVal = 1000;
  int32_t maxDepthVal = 5000;
  uint16_t geometry = 2000;
  std::vector<double> mapped_pivot = {1000, 1500, 4000, 4500, 5000};
  bool lowDepthQuality = true;

  geometry = TMIV::Encoder::plsDepthMapping(minDepthVal, maxDepthVal, piece_num, geometry,
                                            mapped_pivot, lowDepthQuality);

  CHECK(geometry == 4100);
}
