/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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
#include <catch2/generators/catch_generators_range.hpp>

#include "MaxRectPiP.h"

#include <random>

using TMIV::Common::DefaultElement;
using TMIV::Packer::Cluster;
using TMIV::Packer::ClusteringMap;
using TMIV::Packer::MaxRectPiP;

TEST_CASE("TMIV::Packer::MaxRectPiP") {
  const auto viewIdx = 100;
  const auto isBasicView = GENERATE(false, true);
  const auto clusterId = 81;
  const auto entityId = 4;
  CAPTURE(isBasicView);

  auto cluster = Cluster{viewIdx, isBasicView, clusterId, entityId};

  std::mt19937 rnd{2};

  const auto height = 100U;
  const auto width = 150U;

  auto map = ClusteringMap::lumaOnly({width, height});

  for (int32_t n = 0; n < 1000; ++n) {
    const auto i = static_cast<int32_t>(rnd() % height);
    const auto j = static_cast<int32_t>(rnd() % width);
    cluster.push(i, j);
    map.getPlane(0)(i, j) = clusterId;
  }

  const auto pip = GENERATE(false, true);
  CAPTURE(pip);

  auto unit = MaxRectPiP{2 * width, 2 * height, 4, pip};

  auto output = MaxRectPiP::Output{};
  CHECK(unit.push(cluster, map, output));

  CHECK(output.x() == 0);
  CHECK(output.y() == 0);
  CHECK(output.isRotated() != isBasicView);

  SECTION("Second smaller cluster") {
    const auto clusterId2 = clusterId + 3;
    cluster = Cluster{viewIdx, isBasicView, clusterId2, entityId};

    const auto height2 = 10U;
    const auto width2 = 30U;

    for (int32_t n = 0; n < 10; ++n) {
      const auto i = static_cast<int32_t>(rnd() % height2);
      const auto j = static_cast<int32_t>(rnd() % width2);
      cluster.push(i, j);
      map.getPlane(0)(i, j) = clusterId2;
    }

    auto output2 = MaxRectPiP::Output{};
    CHECK(unit.push(cluster, map, output2));

    CHECK(output2.x() == 0);
    CHECK(output2.y() == (isBasicView ? 100 : 152));
    CHECK(output2.isRotated() != isBasicView);
  }
}
