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

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_range.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <TMIV/Renderer/RecoverPrunedViews.h>

#include <random>

using Catch::Matchers::ContainsSubstring;
using TMIV::Common::ColorFormat;
using TMIV::Common::DefaultElement;
using TMIV::Common::Frame;
using TMIV::Common::unusedPatchIdx;
using TMIV::MivBitstream::AccessUnit;
using TMIV::MivBitstream::FlexiblePatchOrientation;
using TMIV::Renderer::recoverPrunedViews;

TEST_CASE("TMIV::Renderer::recoverPrunedViews") {
  auto frame = AccessUnit{};

  GIVEN("a default-initialized frame") {
    WHEN("recovering pruned views") {
      const auto actual = recoverPrunedViews(frame);
      THEN("no pruned views are output") { CHECK(actual.empty()); }
    }
  }

  GIVEN("a frame with default-initialized view parameters") {
    const auto viewCount = GENERATE(size_t{1}, size_t{2}, size_t{10});

    for (size_t i = 0; i < viewCount; ++i) {
      frame.viewParamsList.emplace_back();
    }

    WHEN("recovering pruned views") {
      const auto actual = recoverPrunedViews(frame);
      REQUIRE(actual.size() == viewCount);

      THEN("for each view only a 1 x 1 occupancy map is output") {
        for (size_t i = 0; i < viewCount; ++i) {
          REQUIRE(actual[i].occupancy.getNumberOfPlanes() == 1);
          REQUIRE(actual[i].occupancy.getColorFormat() == ColorFormat::YUV400);
          REQUIRE(actual[i].occupancy.getWidth() == 1);
          REQUIRE(actual[i].occupancy.getHeight() == 1);
          REQUIRE(actual[i].occupancy.getBitDepth() == 8);
          REQUIRE(actual[i].occupancy.getPlane(0)(0, 0) == 0);

          REQUIRE(actual[i].geometry.empty());
          REQUIRE(actual[i].texture.empty());
          REQUIRE(actual[i].transparency.empty());
          REQUIRE(actual[i].packed.empty());
        }
      }
    }
  }

  GIVEN("a frame with default-initialized view parameters and atlas access unit") {
    const auto viewCount = GENERATE(size_t{1}, size_t{3});

    for (size_t i = 0; i < viewCount; ++i) {
      frame.viewParamsList.emplace_back();
    }

    auto &atlas = frame.atlas.emplace_back();

    THEN("recovering pruned views fails because the BTPM is empty") {
      REQUIRE_THROWS_WITH(recoverPrunedViews(frame),
                          ContainsSubstring("!atlas.blockToPatchMap.empty()"));
    }

    GIVEN("an empty BTPM") {
      atlas.blockToPatchMap = Frame<>::lumaOnly({});

      WHEN("recovering pruned views") {
        const auto actual = recoverPrunedViews(frame);

        THEN("for each view only a 1 x 1 occupancy and geometry map is output") {
          REQUIRE(actual.size() == viewCount);

          for (size_t i = 0; i < viewCount; ++i) {
            REQUIRE(actual[i].occupancy.getNumberOfPlanes() == 1);
            REQUIRE(actual[i].occupancy.getColorFormat() == ColorFormat::YUV400);
            REQUIRE(actual[i].occupancy.getWidth() == 1);
            REQUIRE(actual[i].occupancy.getHeight() == 1);
            REQUIRE(actual[i].occupancy.getPlane(0)(0, 0) == 0);

            REQUIRE(actual[i].geometry.getNumberOfPlanes() == 1);
            REQUIRE(actual[i].geometry.getColorFormat() == ColorFormat::YUV400);
            REQUIRE(actual[i].geometry.getWidth() == 1);
            REQUIRE(actual[i].geometry.getHeight() == 1);
            REQUIRE(actual[i].geometry.getBitDepth() == 1);
            REQUIRE(actual[i].geometry.getPlane(0)(0, 0) == 0);

            REQUIRE(actual[i].texture.empty());
            REQUIRE(actual[i].transparency.empty());
            REQUIRE(actual[i].packed.empty());
          }
        }
      }
    }

    GIVEN("the atlas is marked as ancillary") {
      atlas.asps.asps_miv_extension().asme_ancillary_atlas_flag(true);

      WHEN("recovering pruned views") {
        const auto actual = recoverPrunedViews(frame);

        THEN("the atlas is skipped, and no geometry map is created") {
          REQUIRE(actual.size() == viewCount);

          for (size_t i = 0; i < viewCount; ++i) {
            REQUIRE_FALSE(actual[i].occupancy.empty());
            REQUIRE(actual[i].geometry.empty());
            REQUIRE(actual[i].texture.empty());
            REQUIRE(actual[i].transparency.empty());
            REQUIRE(actual[i].packed.empty());
          }
        }
      }
    }

    GIVEN("the atlas with texture and geometry components") {
      const auto geoBitDepth = GENERATE(1U, 13U, 16U);
      const auto texBitDepth = GENERATE(2U, 11U, 16U);
      const auto atlasFrameHeight = 20;
      const auto atlasFrameWidth = 10;
      CAPTURE(geoBitDepth, texBitDepth);

      atlas.geoFrame = Frame<>::lumaOnly({atlasFrameHeight, atlasFrameWidth}, geoBitDepth);
      atlas.texFrame = Frame<>::yuv444({atlasFrameWidth, atlasFrameHeight}, texBitDepth);

      THEN("recovering pruned views fails because the BTPM is empty") {
        REQUIRE_THROWS_WITH(recoverPrunedViews(frame),
                            ContainsSubstring("!atlas.blockToPatchMap.empty()"));
      }

      GIVEN("an empty BTPM") {
        atlas.blockToPatchMap = Frame<>::lumaOnly({});

        WHEN("recovering pruned views") {
          const auto actual = recoverPrunedViews(frame);

          THEN("the components are created with matching bit depth") {
            REQUIRE(actual.size() == viewCount);

            for (size_t i = 0; i < viewCount; ++i) {
              REQUIRE_FALSE(actual[i].occupancy.empty());
              REQUIRE_FALSE(actual[i].geometry.empty());
              REQUIRE_FALSE(actual[i].texture.empty());
              REQUIRE(actual[i].transparency.empty());
              REQUIRE(actual[i].packed.empty());

              REQUIRE(actual[i].occupancy.getBitDepth() == 8);
              REQUIRE(actual[i].geometry.getBitDepth() == geoBitDepth);
              REQUIRE(actual[i].texture.getBitDepth() == texBitDepth);
            }
          }
        }

        GIVEN("the atlas has at least two attributes") {
          const auto attrCount = GENERATE(2, 3);
          const auto traBitDepth = GENERATE(3U, 9U);

          for (int32_t i = 0; i < attrCount; ++i) {
            atlas.attrFrameNF.emplace_back() = Frame<>::lumaOnly({}, traBitDepth);
          }

          WHEN("recovering pruned views") {
            const auto actual = recoverPrunedViews(frame);

            THEN("it is assumed that attrIdx == 1 corresponds to transparency") {
              REQUIRE(actual.size() == viewCount);

              for (size_t i = 0; i < viewCount; ++i) {
                REQUIRE_FALSE(actual[i].transparency.empty());
                REQUIRE(actual[i].transparency.getBitDepth() == traBitDepth);
              }
            }
          }
        }

        GIVEN("the atlas frame size is set accordingly") {
          atlas.asps.asps_frame_width(atlasFrameWidth).asps_frame_height(atlasFrameHeight);

          THEN("recovering pruned views fails because the BTPM does not match") {
            REQUIRE_THROWS_WITH(recoverPrunedViews(frame),
                                ContainsSubstring("== atlas.blockToPatchMap.getWidth()"));
          }

          GIVEN("a BTPM of the right size") {
            atlas.blockToPatchMap = Frame<>::lumaOnly({atlasFrameWidth, atlasFrameHeight});

            THEN("recovering pruned views fails because the patch index (0) is out of range") {
              REQUIRE_THROWS_WITH(recoverPrunedViews(frame), ContainsSubstring("patchIdx <"));
            }

            GIVEN("the BTPM is initialized to unused patch idx") {
              atlas.blockToPatchMap.fillValue(unusedPatchIdx);

              WHEN("recovering pruned views") {
                const auto actual = recoverPrunedViews(frame);

                THEN("the components are created with matching bit depth") {
                  REQUIRE(actual.size() == viewCount);

                  for (size_t i = 0; i < viewCount; ++i) {
                    REQUIRE_FALSE(actual[i].occupancy.empty());
                    REQUIRE_FALSE(actual[i].geometry.empty());
                    REQUIRE_FALSE(actual[i].texture.empty());
                    REQUIRE(actual[i].transparency.empty());
                    REQUIRE(actual[i].packed.empty());

                    REQUIRE(actual[i].occupancy.getBitDepth() == 8);
                    REQUIRE(actual[i].geometry.getBitDepth() == geoBitDepth);
                    REQUIRE(actual[i].texture.getBitDepth() == texBitDepth);
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  GIVEN("a frame with a view, atlas, patch and all components") {
    static constexpr auto atlasFrameWidth = 30;
    static constexpr auto atlasFrameHeight = 20;
    static constexpr auto log2BlockSize = 3;
    static constexpr auto blockSize = 1 << log2BlockSize;

    static constexpr auto geoBitDepth = 9;
    static constexpr auto texBitDepth = 12;
    static constexpr auto traBitDepth = 7;

    static constexpr auto patch2dSizeX = atlasFrameWidth / 2;

    frame.viewParamsList.emplace_back()
        .ci.ci_projection_plane_width_minus1(atlasFrameWidth - 1)
        .ci_projection_plane_height_minus1(atlasFrameHeight - 1);
    frame.viewParamsList.constructViewIdIndex();

    auto &atlas = frame.atlas.emplace_back();

    atlas.asps.asps_frame_width(atlasFrameWidth)
        .asps_frame_height(atlasFrameHeight)
        .asps_log2_patch_packing_block_size(log2BlockSize);

    atlas.blockToPatchMap = Frame<>::lumaOnly({(atlasFrameWidth + blockSize - 1) / blockSize,
                                               (atlasFrameHeight + blockSize - 1) / blockSize});

    auto &patch = atlas.patchParamsList.emplace_back()
                      .atlasPatchOrientationIndex(FlexiblePatchOrientation::FPO_NULL)
                      .atlasPatch2dSizeX(patch2dSizeX)
                      .atlasPatch2dSizeY(atlasFrameHeight);

    atlas.occFrame = Frame<bool>::lumaOnly({atlasFrameWidth, atlasFrameHeight});
    atlas.geoFrame = Frame<>::lumaOnly({atlasFrameWidth, atlasFrameHeight}, geoBitDepth);
    atlas.texFrame = Frame<>::yuv444({atlasFrameWidth, atlasFrameHeight}, texBitDepth);

    atlas.attrFrameNF.emplace_back();
    atlas.attrFrameNF.emplace_back() =
        Frame<>::lumaOnly({atlasFrameWidth, atlasFrameHeight}, traBitDepth);

    auto sample = [rnd = std::mt19937{1}](int32_t bitDepth) mutable {
      return static_cast<DefaultElement>(rnd() % (1U << bitDepth));
    };

    for (int32_t i = 0; i < atlasFrameHeight; ++i) {
      for (int32_t j = 0; j < atlasFrameWidth; ++j) {
        atlas.occFrame.getPlane(0)(i, j) = sample(1) == 1;
        atlas.geoFrame.getPlane(0)(i, j) = sample(geoBitDepth);
        atlas.texFrame.getPlane(0)(i, j) = sample(texBitDepth);
        atlas.texFrame.getPlane(1)(i, j) = sample(texBitDepth);
        atlas.texFrame.getPlane(2)(i, j) = sample(texBitDepth);
        atlas.attrFrameNF[1].getPlane(0)(i, j) = sample(traBitDepth);
      }
    }

    WHEN("recovering pruned views") {
      const auto actual = recoverPrunedViews(frame);

      THEN("The pruned view is the input masked by the occupancy map") {
        REQUIRE(actual.size() == 1);
        const auto &prunedView = actual.front();
        REQUIRE(prunedView.occupancy.getSize() == atlas.occFrame.getSize());

        for (int32_t i = 0; i < atlasFrameHeight; ++i) {
          for (int32_t j = 0; j < atlasFrameWidth; ++j) {
            if (atlas.occFrame.getPlane(0)(i, j) && j < patch2dSizeX) {
              REQUIRE(prunedView.occupancy.getPlane(0)(i, j) == 255);
              REQUIRE(prunedView.geometry.getPlane(0)(i, j) == atlas.geoFrame.getPlane(0)(i, j));
              REQUIRE(prunedView.texture.getPlane(0)(i, j) == atlas.texFrame.getPlane(0)(i, j));
              REQUIRE(prunedView.texture.getPlane(1)(i, j) == atlas.texFrame.getPlane(1)(i, j));
              REQUIRE(prunedView.texture.getPlane(2)(i, j) == atlas.texFrame.getPlane(2)(i, j));
              REQUIRE(prunedView.transparency.getPlane(0)(i, j) ==
                      atlas.attrFrameNF[1].getPlane(0)(i, j));
            } else {
              REQUIRE(prunedView.occupancy.getPlane(0)(i, j) == 0);
              REQUIRE(prunedView.geometry.getPlane(0)(i, j) == 0);
              REQUIRE(prunedView.texture.getPlane(0)(i, j) == prunedView.texture.neutralValue());
              REQUIRE(prunedView.texture.getPlane(1)(i, j) == prunedView.texture.neutralValue());
              REQUIRE(prunedView.texture.getPlane(2)(i, j) == prunedView.texture.neutralValue());
              REQUIRE(prunedView.transparency.getPlane(0)(i, j) == 0);
            }
          }
        }
      }
    }

    GIVEN("that the atlas has constant depth") {
      const auto offsetD = GENERATE(DefaultElement{4}, DefaultElement{7});

      atlas.asps.asps_miv_extension().asme_patch_constant_depth_flag(true);
      patch.atlasPatch3dOffsetD(offsetD);

      WHEN("recovering pruned views") {
        const auto actual = recoverPrunedViews(frame);

        THEN("The geometry of the pruned view is not based on the geometry map") {
          REQUIRE(actual.size() == 1);
          const auto &prunedView = actual.front();
          REQUIRE(prunedView.occupancy.getSize() == atlas.occFrame.getSize());

          for (int32_t i = 0; i < atlasFrameHeight; ++i) {
            for (int32_t j = 0; j < atlasFrameWidth; ++j) {
              if (atlas.occFrame.getPlane(0)(i, j) && j < patch2dSizeX) {
                REQUIRE(prunedView.occupancy.getPlane(0)(i, j) == 255);
                REQUIRE(prunedView.geometry.getPlane(0)(i, j) == offsetD);
                REQUIRE(prunedView.texture.getPlane(0)(i, j) == atlas.texFrame.getPlane(0)(i, j));
                REQUIRE(prunedView.texture.getPlane(1)(i, j) == atlas.texFrame.getPlane(1)(i, j));
                REQUIRE(prunedView.texture.getPlane(2)(i, j) == atlas.texFrame.getPlane(2)(i, j));
                REQUIRE(prunedView.transparency.getPlane(0)(i, j) ==
                        atlas.attrFrameNF[1].getPlane(0)(i, j));
              } else {
                REQUIRE(prunedView.occupancy.getPlane(0)(i, j) == 0);
                REQUIRE(prunedView.geometry.getPlane(0)(i, j) == 0);
                REQUIRE(prunedView.texture.getPlane(0)(i, j) == prunedView.texture.neutralValue());
                REQUIRE(prunedView.texture.getPlane(1)(i, j) == prunedView.texture.neutralValue());
                REQUIRE(prunedView.texture.getPlane(2)(i, j) == prunedView.texture.neutralValue());
                REQUIRE(prunedView.transparency.getPlane(0)(i, j) == 0);
              }
            }
          }
        }
      }
    }
  }
}
