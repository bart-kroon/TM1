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
#include <catch2/generators/catch_generators_range.hpp>

#include <TMIV/Packer/Cluster.h>

#include <random>
#include <vector>

using TMIV::Common::DefaultElement;
using TMIV::Packer::Cluster;
using TMIV::Packer::ClusteringMap;

TEST_CASE("TMIV::Packer::Cluster::push and accessors") {
  const auto viewIdx = GENERATE(3, 100);
  const auto isBasicView = GENERATE(false, true);
  const auto clusterId = GENERATE(0, 81);
  const auto entityId = GENERATE(0, 4);
  CAPTURE(viewIdx, isBasicView, clusterId, entityId);

  auto cluster = Cluster{viewIdx, isBasicView, clusterId, entityId};

  std::mt19937 rnd{2};

  const auto height = 100U;
  const auto width = 150U;

  for (int32_t i = 0; i < 1000; ++i) {
    cluster.push(13 + static_cast<int32_t>(rnd() % height),
                 16 + static_cast<int32_t>(rnd() % width));
  }

  CHECK(cluster.getViewIdx() == viewIdx);
  CHECK(cluster.getClusterId() == clusterId);
  CHECK(cluster.getEntityId() == entityId);
  CHECK(cluster.getNumActivePixels() == 1000);
  CHECK(cluster.imin() == 13);
  CHECK(cluster.jmin() == 16);
  CHECK(cluster.imax() == 112);
  CHECK(cluster.jmax() == 165);
  CHECK(cluster.width() == 150);
  CHECK(cluster.height() == 100);
  CHECK(cluster.getArea() == 15000);
  CHECK(cluster.getMinSize() == 100);
  CHECK(cluster.isBasicView() == isBasicView);

  SECTION("TMIV::Packer::Cluster::numActivePixels") {
    cluster.numActivePixels() = clusterId;
    CHECK(cluster.getNumActivePixels() == clusterId);
  }

  SECTION("TMIV::Packer::Cluster::setEntityId") {
    const auto newCluster = Cluster::setEntityId(cluster, 77);

    CHECK(newCluster.getViewIdx() == viewIdx);
    CHECK(newCluster.getClusterId() == clusterId);
    CHECK(newCluster.getEntityId() == 77);
    CHECK(newCluster.getNumActivePixels() == 1000);
    CHECK(newCluster.imin() == 13);
    CHECK(newCluster.jmin() == 16);
    CHECK(newCluster.imax() == 112);
    CHECK(newCluster.jmax() == 165);
    CHECK(newCluster.width() == 150);
    CHECK(newCluster.height() == 100);
    CHECK(newCluster.getArea() == 15000);
    CHECK(newCluster.getMinSize() == 100);
    CHECK(newCluster.isBasicView() == isBasicView);
  }

  SECTION("TMIV::Packer::Cluster::align") {
    const auto newCluster = Cluster::align(cluster, 37);

    CHECK(newCluster.getViewIdx() == viewIdx);
    CHECK(newCluster.getClusterId() == clusterId);
    CHECK(newCluster.getEntityId() == entityId);
    CHECK(newCluster.getNumActivePixels() == 1000);
    CHECK(newCluster.imin() == 0);
    CHECK(newCluster.jmin() == 0);
    CHECK(newCluster.imax() == 112);
    CHECK(newCluster.jmax() == 165);
    CHECK(newCluster.width() == 166);
    CHECK(newCluster.height() == 113);
    CHECK(newCluster.getArea() == 18758);
    CHECK(newCluster.getMinSize() == 113);
    CHECK(newCluster.isBasicView() == isBasicView);
  }

  SECTION("TMIV::Packer::Cluster::split") {
    if (cluster.isBasicView()) {
      return; // Splitting of basic views is out of contract
    }

    auto map = ClusteringMap::lumaOnly({cluster.imax() + 1, cluster.jmax() + 1});
    map.fillNeutral();

    for (int32_t i = 0; i < map.getHeight(); ++i) {
      for (int32_t j = 0; j < map.getWidth(); ++j) {
        map.getPlane(0)(i, j) = static_cast<DefaultElement>(rnd() % 8U == 0 ? clusterId : 777);
      }
    }

    const auto overlap = 4;
    const auto [c1, c2] = cluster.split(map, overlap);

    CHECK(c1.getViewIdx() == viewIdx);
    CHECK(c1.getClusterId() == clusterId);
    CHECK(c1.getEntityId() == entityId);
    CHECK(c1.getNumActivePixels() == 985);
    CHECK(c1.imin() == 13);
    CHECK(c1.jmin() == 16);
    CHECK(c1.imax() == 112);
    CHECK(c1.jmax() == 93);
    CHECK(c1.width() == 78);
    CHECK(c1.height() == 100);
    CHECK(c1.getArea() == 7800);
    CHECK(c1.getMinSize() == 78);
    CHECK(c1.isBasicView() == isBasicView);

    CHECK(c2.getViewIdx() == viewIdx);
    CHECK(c2.getClusterId() == clusterId);
    CHECK(c2.getEntityId() == entityId);
    CHECK(c2.getNumActivePixels() == 981);
    CHECK(c1.imin() == 13);
    CHECK(c1.jmin() == 16);
    CHECK(c1.imax() == 112);
    CHECK(c1.jmax() == 93);
    CHECK(c2.width() == 80);
    CHECK(c2.height() == 100);
    CHECK(c2.getArea() == 8000);
    CHECK(c2.getMinSize() == 80);
    CHECK(c2.isBasicView() == isBasicView);

    SECTION("TMIV::Packer::Cluster::merge") {
      const auto c3 = Cluster::merge(c1, c2);
      CHECK(c3.getViewIdx() == viewIdx);
      CHECK(c3.getClusterId() == clusterId);
      CHECK(c3.getEntityId() == entityId);
      CHECK(c3.getNumActivePixels() == 1966);
      CHECK(c3.imin() == 13);
      CHECK(c3.jmin() == 16);
      CHECK(c3.imax() == 112);
      CHECK(c3.jmax() == 165);
      CHECK(c3.width() == 150);
      CHECK(c3.height() == 100);
      CHECK(c3.getArea() == 15000);
      CHECK(c3.getMinSize() == 100);
      CHECK(c3.isBasicView() == isBasicView);
    }
  }
}

TEST_CASE("TMIV::Packer::Cluster::recursiveSplit") {
  SECTION("Two distant samples") {
    const auto width = GENERATE(30, 70, 120);
    const auto height = GENERATE(30, 70, 120);
    CAPTURE(width, height);

    Cluster unit{};
    unit.push(1, 1);
    unit.push(height, width);

    auto map = ClusteringMap::lumaOnly({128, 128}); // Has to be at least the size of the cluster
    map.fillNeutral();

    std::vector<Cluster> out_clusters{};
    const int32_t alignment = 2;
    const int32_t minPatchSize = 4;

    const auto expected = [=]() { return width == 30 && height == 30 ? 1U : 2U; }();

    unit.recursiveSplit(map, out_clusters, alignment, minPatchSize);
    CHECK(out_clusters.size() == expected);
  }

  SECTION("L-shapes") {
    const auto width = GENERATE(30, 70, 120);
    const auto height = GENERATE(30, 70, 120);
    const auto variation = GENERATE(0, 1, 2, 3);
    CAPTURE(width, height, variation);

    Cluster unit{};

    auto map = ClusteringMap::lumaOnly({128, 128}); // Has to be at least the size of the cluster
    map.fillNeutral();

    for (int32_t i = 1; i <= height; ++i) {
      const auto j = variation % 2 == 0 ? width : 1;
      unit.push(i, j);
      map.getPlane(0)(i, j) = static_cast<DefaultElement>(unit.getClusterId());
    }
    for (int32_t j = 1; j <= width; ++j) {
      const auto i = variation % 2 == 1 ? height : 1;
      unit.push(i, j);
      map.getPlane(0)(i, j) = static_cast<DefaultElement>(unit.getClusterId());
    }

    std::vector<Cluster> out_clusters{};
    const int32_t alignment = 2;
    const int32_t minPatchSize = 4;

    const auto expected = [=]() {
      if ((height == 120 && width == 30) || (height == 30 && width == 120)) {
        return 2U;
      }
      if ((height == 120 && width != 30) || (width == 120 && height != 30)) {
        return 3U;
      }
      if (height == 70 && width == 70) {
        return 3U;
      }
      if (height == 30 && width == 30) {
        return 1U;
      }
      return 2U;
    }();

    unit.recursiveSplit(map, out_clusters, alignment, minPatchSize);
    CHECK(out_clusters.size() == expected);
  }

  SECTION("C-shapes") {
    const auto width = GENERATE(30, 70, 120);
    const auto height = GENERATE(30, 70, 120);
    const auto variation = GENERATE(0, 1, 2, 3);
    CAPTURE(width, height, variation);

    Cluster unit{};

    auto map = ClusteringMap::lumaOnly({128, 128}); // Has to be at least the size of the cluster
    map.fillNeutral();

    for (int32_t i = 1; i <= height; ++i) {
      if (variation != 0) {
        unit.push(i, 1);
        map.getPlane(0)(i, 1) = static_cast<DefaultElement>(unit.getClusterId());
      }
      if (variation != 1) {
        unit.push(i, width);
        map.getPlane(0)(i, 1) = static_cast<DefaultElement>(unit.getClusterId());
      }
    }

    for (int32_t j = 1; j <= width; ++j) {
      if (variation != 2) {
        unit.push(1, j);
      }
      if (variation != 3) {
        unit.push(height, j);
      }
    }

    std::vector<Cluster> out_clusters{};
    const int32_t alignment = 2;
    const int32_t minPatchSize = 4;

    const auto expected = [=]() {
      if (width == 30 && height == 30) {
        return 1U;
      }
      return 2U;
    }();

    unit.recursiveSplit(map, out_clusters, alignment, minPatchSize);
    CHECK(out_clusters.size() == expected);
  }
}
