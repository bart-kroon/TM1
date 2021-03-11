/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
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

#include <catch2/catch.hpp>

#include <TMIV/Packer/Retriever.h>

namespace TMIV::Packer {
namespace {
const uint16_t INVALID = (1 << 16) - 1;

void addRectangle(Common::Mask &mask, Common::Vec2i topLeft, Common::Vec2i bottomRight) {
  for (int x = topLeft.x(); x <= bottomRight.x(); ++x) {
    for (int y = topLeft.y(); y <= bottomRight.y(); ++y) {
      mask.getPlane(0)(y, x) = 1;
    }
  }
}
} // namespace

SCENARIO("Cluster retrieving") {
  const int viewId = 0;
  const int firstClusterId = 0;
  bool isBasicView{};
  bool enableMerging{};
  GIVEN("a 0x0 mask") {
    const Common::Mask mask{};
    WHEN("retrieving clusters") {
      const auto [clusterList, clusteringMap] =
          retrieveClusters(viewId, mask, firstClusterId, isBasicView, enableMerging);
      THEN("return empty cluster list and map") {
        REQUIRE(clusterList.empty());
        REQUIRE(clusteringMap.getNumberOfPlanes() == 1);
        REQUIRE(clusteringMap.getWidth() == 0);
        REQUIRE(clusteringMap.getHeight() == 0);
      }
    }
  }

  GIVEN("a 2x2 mask with only zeroes") {
    Common::Mask mask{};
    mask.resize(2, 2);
    WHEN("retrieving clusters") {
      const auto [clusterList, clusteringMap] =
          retrieveClusters(viewId, mask, firstClusterId, isBasicView, enableMerging);
      THEN("return empty cluster list and map with max value") {
        REQUIRE(clusterList.empty());
        REQUIRE(clusteringMap.getNumberOfPlanes() == 1);
        REQUIRE(clusteringMap.getWidth() == 2);
        REQUIRE(clusteringMap.getHeight() == 2);
        for (const auto pixel : clusteringMap.getPlane(0)) {
          REQUIRE(pixel == (1 << 16) - 1);
        }
      }
    }
  }

  GIVEN("a 2x2 mask with nonzero values on the diagonal") {
    Common::Mask mask{};
    mask.resize(2, 2);
    mask.getPlane(0)(0, 0) = 1;
    mask.getPlane(0)(1, 1) = 1;
    WHEN("retrieving clusters") {
      const auto [clusterList, clusteringMap] =
          retrieveClusters(viewId, mask, firstClusterId, isBasicView, enableMerging);
      THEN("return one cluster and map with zeroes on the diagonal") {
        REQUIRE(clusterList.size() == 1);
        REQUIRE(!clusterList[0].isBasicView());
        REQUIRE(clusterList[0].getNumActivePixels() == 2);
        REQUIRE(clusterList[0].getEntityId() == 0);

        REQUIRE(clusteringMap.getNumberOfPlanes() == 1);
        REQUIRE(clusteringMap.getWidth() == 2);
        REQUIRE(clusteringMap.getHeight() == 2);
        const auto plane = clusteringMap.getPlane(0);
        REQUIRE(plane(0, 0) == 0);
        REQUIRE(plane(1, 0) == (1 << 16) - 1);
        REQUIRE(plane(0, 1) == (1 << 16) - 1);
        REQUIRE(plane(1, 1) == 0);
      }
    }
  }

  GIVEN("a 5x5 mask with a 3x3 nonzero cluster in the center values on the diagonal and merging "
        "enabled") {
    enableMerging = true;
    Common::Mask mask{};
    mask.resize(5, 5);
    addRectangle(mask, {1, 1}, {3, 3});
    WHEN("retrieving clusters") {
      const auto [clusterList, clusteringMap] =
          retrieveClusters(viewId, mask, firstClusterId, isBasicView, enableMerging);
      THEN("return one cluster and map with zeroes on the diagonal") {
        REQUIRE(clusterList.size() == 1);
        REQUIRE(!clusterList[0].isBasicView());
        REQUIRE(clusterList[0].getNumActivePixels() == 9);
        REQUIRE(clusterList[0].getEntityId() == 0);

        REQUIRE(clusteringMap.getNumberOfPlanes() == 1);
        REQUIRE(clusteringMap.getWidth() == 5);
        REQUIRE(clusteringMap.getHeight() == 5);
        const auto plane = clusteringMap.getPlane(0);
        REQUIRE(plane(0, 0) == INVALID);
        REQUIRE(plane(1, 0) == INVALID);
        REQUIRE(plane(0, 1) == INVALID);
        REQUIRE(plane(1, 1) == 0);
      }
    }
  }

  GIVEN("a 10x10 mask with two clusters") {
    Common::Mask mask{};
    mask.resize(10, 10);
    addRectangle(mask, {1, 1}, {2, 2});
    addRectangle(mask, {6, 6}, {7, 7});
    WHEN("retrieving clusters") {
      const auto [clusterList, clusteringMap] =
          retrieveClusters(viewId, mask, firstClusterId, isBasicView, enableMerging);
      THEN("return two clusters") {
        REQUIRE(clusterList.size() == 2);
        REQUIRE(!clusterList[0].isBasicView());
        REQUIRE(clusterList[0].getNumActivePixels() == 4);
        REQUIRE(!clusterList[1].isBasicView());
        REQUIRE(clusterList[1].getNumActivePixels() == 4);
      }
    }
  }

  GIVEN("a 20x20 mask with two overlapping clusters and merging enabled") {
    enableMerging = true;
    Common::Mask mask{};
    mask.resize(20, 20);
    addRectangle(mask, {1, 1}, {10, 3});
    addRectangle(mask, {1, 4}, {3, 10});
    addRectangle(mask, {6, 6}, {7, 7});
    WHEN("retrieving clusters") {
      const auto [clusterList, clusteringMap] =
          retrieveClusters(viewId, mask, firstClusterId, isBasicView, enableMerging);
      THEN("return one cluster") { REQUIRE(clusterList.size() == 1); }
    }
  }
}
} // namespace TMIV::Packer
