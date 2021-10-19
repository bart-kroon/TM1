/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#include <TMIV/ViewOptimizer/ServerSideInpainter.h>

#include <TMIV/Common/Factory.h>
#include <TMIV/Renderer/IInpainter.h>
#include <TMIV/Renderer/ISynthesizer.h>

#include <cstring>

using namespace std::string_literals;

namespace {
class FakeOptimizer : public TMIV::ViewOptimizer::IViewOptimizer {
public:
  FakeOptimizer(const TMIV::Common::Json & /* rootNode */,
                const TMIV::Common::Json & /* componentNode */) {
    constructed = true;
  }

  // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
  static inline bool constructed = false;

  auto optimizeParams(const TMIV::ViewOptimizer::SourceParams &params)
      -> TMIV::ViewOptimizer::ViewOptimizerParams override {
    m_params = {params.viewParamsList};
    for (auto &vp : m_params.viewParamsList) {
      vp.name = "transport";
    }
    return m_params;
  }

  [[nodiscard]] auto optimizeFrame(TMIV::Common::MVD16Frame views) const
      -> TMIV::Common::MVD16Frame override {
    return views;
  }

private:
  TMIV::ViewOptimizer::ViewOptimizerParams m_params;
};

class FakeSynthesizer : public TMIV::Renderer::ISynthesizer {
public:
  FakeSynthesizer(const TMIV::Common::Json & /* rootNode */,
                  const TMIV::Common::Json & /* componentNode */) {
    constructed = true;
  }

  // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
  static inline bool constructed = false;

  static inline std::function<void(const TMIV::MivBitstream::AccessUnit &frame,
                                   const TMIV::MivBitstream::CameraConfig &viewportParams)>
      inspect_renderFrame_input; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

  [[nodiscard]] auto renderFrame(const TMIV::MivBitstream::AccessUnit &frame,
                                 const TMIV::MivBitstream::CameraConfig &viewportParams) const
      -> TMIV::Common::Texture444Depth16Frame override {
    if (inspect_renderFrame_input) {
      inspect_renderFrame_input(frame, viewportParams);
    }

    auto output = TMIV::Common::Texture444Depth16Frame{};
    output.first.createYuv444(
        {viewportParams.viewParams.ci.ci_projection_plane_width_minus1() + 1,
         viewportParams.viewParams.ci.ci_projection_plane_height_minus1() + 1},
        viewportParams.bitDepthColor);
    output.first.getPlane(0)[0] = 456;
    output.second.createY({viewportParams.viewParams.ci.ci_projection_plane_width_minus1() + 1,
                           viewportParams.viewParams.ci.ci_projection_plane_height_minus1() + 1},
                          viewportParams.bitDepthDepth);
    return output;
  }
};

class FakeInpainter : public TMIV::Renderer::IInpainter {
public:
  FakeInpainter(const TMIV::Common::Json & /* rootNode */,
                const TMIV::Common::Json & /* componentNode */) {
    constructed = true;
  }

  // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
  static inline bool constructed = false;

  // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
  static inline bool called = false;

  void inplaceInpaint(TMIV::Common::Texture444Depth16Frame & /* viewport */,
                      const TMIV::MivBitstream::ViewParams & /* metadata */) const override {
    called = true;
  }
};

const auto construct = []() {
  return TMIV::ViewOptimizer::ServerSideInpainter{TMIV::Common::Json{},
                                                  TMIV::Common::Json::parse(R"({
    "SubMethod": "FakeOptimizer",
    "FakeOptimizer": {},
    "resolution": [ 200, 100 ],
    "SynthesizerMethod": "FakeSynthesizer",
    "FakeSynthesizer": {},
    "InpainterMethod": "FakeInpainter",
    "FakeInpainter": {},
    "blurRadius": 12,
    "inpaintThreshold": 100,
    "fieldOfViewMargin" : 0.3
}
)"s)};
};

const auto sourceParams = []() {
  auto params = TMIV::ViewOptimizer::SourceParams{};

  auto &vp_1 = params.viewParamsList.emplace_back();
  vp_1.ci.ci_projection_plane_width_minus1(1023)
      .ci_projection_plane_height_minus1(511)
      .ci_cam_type(TMIV::MivBitstream::CiCamType::orthographic)
      .ci_ortho_width(89.F)
      .ci_ortho_height(55.F);
  vp_1.pose.position = {1.F, 2.F, 3.F};
  vp_1.dq.dq_norm_disp_low(0.02F).dq_norm_disp_high(1.F);
  vp_1.viewId = TMIV::MivBitstream::ViewId{0};

  auto &vp_2 = params.viewParamsList.emplace_back();
  vp_2.ci.ci_projection_plane_width_minus1(255)
      .ci_projection_plane_height_minus1(511)
      .ci_cam_type(TMIV::MivBitstream::CiCamType::perspective)
      .ci_perspective_center_hor(1.F)
      .ci_perspective_center_ver(2.F)
      .ci_perspective_focal_hor(3.F)
      .ci_perspective_focal_ver(4.F);
  vp_2.pose.position = {10.F, -5.F, 3.F};
  vp_2.dq.dq_norm_disp_low(-0.03F).dq_norm_disp_high(0.5F);
  vp_2.viewId = TMIV::MivBitstream::ViewId{1};

  for (auto &vp : params.viewParamsList) {
    vp.name = "source";
  }

  params.depthLowQualityFlag = false;

  params.viewParamsList.constructViewIdIndex();
  return params;
};

const auto inputFrame = [](const TMIV::ViewOptimizer::SourceParams &params) {
  REQUIRE(!params.viewParamsList.empty());
  auto frame = TMIV::Common::MVD16Frame{};

  std::transform(params.viewParamsList.cbegin(), params.viewParamsList.cend(),
                 std::back_inserter(frame), [](const TMIV::MivBitstream::ViewParams &vp) {
                   auto frame = TMIV::Common::TextureDepth16Frame{};
                   frame.texture.createYuv420({vp.ci.ci_projection_plane_width_minus1() + 1,
                                               vp.ci.ci_projection_plane_height_minus1() + 1},
                                              10);
                   frame.depth.createY({vp.ci.ci_projection_plane_width_minus1() + 1,
                                        vp.ci.ci_projection_plane_height_minus1() + 1},
                                       16);
                   return frame;
                 });
  return frame;
};
} // namespace

TEST_CASE("ServerSideInpainter") {
  // Factory registration
  TMIV::Common::Factory<TMIV::ViewOptimizer::IViewOptimizer>::getInstance()
      .registerAs<TMIV::ViewOptimizer::ServerSideInpainter>("ServerSideInpainter"s);
  TMIV::Common::Factory<TMIV::ViewOptimizer::IViewOptimizer>::getInstance()
      .registerAs<FakeOptimizer>("FakeOptimizer"s);
  TMIV::Common::Factory<TMIV::Renderer::ISynthesizer>::getInstance().registerAs<FakeSynthesizer>(
      "FakeSynthesizer"s);
  TMIV::Common::Factory<TMIV::Renderer::IInpainter>::getInstance().registerAs<FakeInpainter>(
      "FakeInpainter"s);

  SECTION("Factory creation") {
    const auto rootNode = TMIV::Common::Json{};
    const auto componentNode = TMIV::Common::Json::parse(R"({
    "ViewOptimizerMethod": "ServerSideInpainter",
    "ServerSideInpainter": {
        "SubMethod": "FakeOptimizer",
        "FakeOptimizer": {},
        "resolution": [ 1024, 512 ],
        "SynthesizerMethod": "FakeSynthesizer",
        "FakeSynthesizer": {},
        "InpainterMethod": "FakeInpainter",
        "FakeInpainter": {},
        "blurRadius": 12,
        "inpaintThreshold": 100,
        "fieldOfViewMargin" : 0.3
    }
}
)"s);
    auto ssi = TMIV::Common::create<TMIV::ViewOptimizer::IViewOptimizer>("ViewOptimizer"s, rootNode,
                                                                         componentNode);
    static_assert(
        std::is_same_v<decltype(ssi), std::unique_ptr<TMIV::ViewOptimizer::IViewOptimizer>>);
    REQUIRE_NOTHROW(dynamic_cast<TMIV::ViewOptimizer::ServerSideInpainter &>(*ssi));
  }

  SECTION("Construction") { construct(); }

  SECTION("The sub method is constructed") {
    FakeOptimizer::constructed = false;
    construct();
    REQUIRE(FakeOptimizer::constructed);
  }

  SECTION("The synthesizer is constructed") {
    FakeSynthesizer::constructed = false;
    construct();
    REQUIRE(FakeSynthesizer::constructed);
  }

  SECTION("The inpainter is constructed") {
    FakeInpainter::constructed = false;
    construct();
    REQUIRE(FakeInpainter::constructed);
  }

  GIVEN("A server-side inpainter and typical parameters") {
    auto ssi = construct();
    const auto inParams = sourceParams();

    FakeInpainter::called = false;

    WHEN("optimizing the parameters") {
      auto params = ssi.optimizeParams(inParams);

      THEN("SSI adds one view to the VPL") {
        REQUIRE(params.viewParamsList.size() == inParams.viewParamsList.size() + 1);
      }

      THEN("The added view has configurable projection plane size") {
        REQUIRE(params.viewParamsList.back().ci.ci_projection_plane_width_minus1() + 1 == 200);
        REQUIRE(params.viewParamsList.back().ci.ci_projection_plane_height_minus1() + 1 == 100);
      }

      THEN("The added view is full ERP") {
        REQUIRE(params.viewParamsList.back().ci.ci_cam_type() ==
                TMIV::MivBitstream::CiCamType::equirectangular);
        REQUIRE(params.viewParamsList.back().ci.ci_erp_phi_min() == -180.F);
        REQUIRE(params.viewParamsList.back().ci.ci_erp_phi_max() == 180.F);
        REQUIRE(params.viewParamsList.back().ci.ci_erp_theta_min() == -90.F);
        REQUIRE(params.viewParamsList.back().ci.ci_erp_theta_max() == 90.F);
      }

      THEN("The transport views are forwarded") {
        for (size_t i = 0; i + 1 < params.viewParamsList.size(); ++i) {
          REQUIRE(params.viewParamsList[i].name == "transport"s);
        }
      }

      THEN("The added view is forward and upright") {
        REQUIRE(params.viewParamsList.back().pose.orientation == TMIV::Common::neutralOrientationD);
      }

      THEN("The cardinal point of the added view is at the center of gravity") {
        REQUIRE(params.viewParamsList.back().pose.position.x() == 5.5F);
        REQUIRE(params.viewParamsList.back().pose.position.y() == -1.5F);
        REQUIRE(params.viewParamsList.back().pose.position.z() == 3.F);
      }

      THEN("The added view is an additional view") {
        REQUIRE(!params.viewParamsList.back().isBasicView);
      }

      THEN("The added view has a depth range that includes the source depth ranges") {
        const auto &vp = params.viewParamsList.back();

        for (const auto &vpIn : inParams.viewParamsList) {
          REQUIRE(vp.dq.dq_norm_disp_low() <= vpIn.dq.dq_norm_disp_low());
          REQUIRE(vpIn.dq.dq_norm_disp_low() < vpIn.dq.dq_norm_disp_high());
          REQUIRE(vpIn.dq.dq_norm_disp_high() <= vp.dq.dq_norm_disp_high());
        }
      }

      THEN("Only the added view is marked as inpainted") {
        REQUIRE(params.viewParamsList.back().isInpainted);
        for (auto i = params.viewParamsList.crbegin() + 1; i != params.viewParamsList.crend();
             ++i) {
          REQUIRE(!i->isInpainted);
        }
      }

      AND_WHEN("Optimizing a frame") {
        const auto inFrame = inputFrame(inParams);
        auto outFrame = ssi.optimizeFrame(inFrame);

        THEN("SSI adds one view to the frame") { REQUIRE(outFrame.size() == inFrame.size() + 1); }

        THEN("The added view has the configured size for texture and depth") {
          REQUIRE(outFrame.back().texture.getWidth() == 200);
          REQUIRE(outFrame.back().texture.getHeight() == 100);
          REQUIRE(outFrame.back().depth.getWidth() == 200);
          REQUIRE(outFrame.back().depth.getHeight() == 100);
        }

        THEN("The inpainter is called") { REQUIRE(FakeInpainter::called); }
      }
    }
  }

  GIVEN("A server-side inpainter and only perspective views") {
    auto ssi = construct();

    auto inParams = sourceParams();
    inParams.viewParamsList.front().ci = inParams.viewParamsList.back().ci;

    FakeInpainter::called = false;

    WHEN("optimizing the parameters") {
      auto params = ssi.optimizeParams(inParams);

      THEN("The added view is partial ERP") {
        REQUIRE(params.viewParamsList.back().ci.ci_cam_type() ==
                TMIV::MivBitstream::CiCamType::equirectangular);
        REQUIRE(params.viewParamsList.back().ci.ci_erp_phi_min() == Approx(-115.25447F));
        REQUIRE(params.viewParamsList.back().ci.ci_erp_phi_max() == Approx(115.25447F));
        REQUIRE(params.viewParamsList.back().ci.ci_erp_theta_min() == -90.F);
        REQUIRE(params.viewParamsList.back().ci.ci_erp_theta_max() == 90.F);
      }
    }
  }

  GIVEN("An interceptable renderFrame call") {
    auto renderParameters = std::optional<
        std::tuple<TMIV::MivBitstream::AccessUnit, TMIV::MivBitstream::CameraConfig>>{};

    FakeSynthesizer::inspect_renderFrame_input =
        [&renderParameters](const TMIV::MivBitstream::AccessUnit &frame,
                            const TMIV::MivBitstream::CameraConfig &viewportParams) {
          renderParameters = std::tuple{frame, viewportParams};
        };

    WHEN("Using typical inputs for the server-side inpainter") {
      auto ssi = construct();
      const auto inParams = sourceParams();
      const auto inFrame = inputFrame(inParams);
      const auto outParams = ssi.optimizeParams(inParams);
      const auto outFrame = ssi.optimizeFrame(inFrame);

      FakeSynthesizer::inspect_renderFrame_input = nullptr;

      THEN("The added view is synthesized") {
        REQUIRE(renderParameters.has_value());
        const auto &renderFrame = std::get<0>(*renderParameters);

        AND_THEN("The added view is synthesized from the input views") {
          REQUIRE(std::all_of(
              renderFrame.viewParamsList.cbegin(), renderFrame.viewParamsList.cend(),
              [](const TMIV::MivBitstream::ViewParams &vp) { return vp.name == "source"s; }));
          REQUIRE(renderFrame.viewParamsList.size() == inParams.viewParamsList.size());
          REQUIRE(renderFrame.viewParamsList == inParams.viewParamsList);

          REQUIRE(outFrame.back().texture.getPlane(0)[0] == 456);
        }

        AND_THEN("The access unit that is input to the synthesizer has one patch per atlas") {
          for (const auto &atlas : renderFrame.atlas) {
            REQUIRE(atlas.patchParamsList.size() == 1);
          }
        }

        AND_THEN("The patch in the access unit corresponds to a full view") {
          for (size_t i = 0; i < renderFrame.atlas.size(); ++i) {
            const auto &pp = renderFrame.atlas[i].patchParamsList.front();
            REQUIRE(TMIV::MivBitstream::ViewId{i} == pp.atlasPatchProjectionId());
            REQUIRE(pp.atlasPatchOrientationIndex() ==
                    TMIV::MivBitstream::FlexiblePatchOrientation::FPO_NULL);
            REQUIRE(pp.atlasPatch2dPosX() == 0);
            REQUIRE(pp.atlasPatch2dPosY() == 0);
            REQUIRE(pp.atlasPatch2dSizeX() ==
                    inParams.viewParamsList[i].ci.ci_projection_plane_width_minus1() + 1);
            REQUIRE(pp.atlasPatch2dSizeY() ==
                    inParams.viewParamsList[i].ci.ci_projection_plane_height_minus1() + 1);
            REQUIRE(pp.atlasPatch3dOffsetU() == 0);
            REQUIRE(pp.atlasPatch3dOffsetV() == 0);
            REQUIRE(pp.atlasPatch3dOffsetD() == 0);
            REQUIRE(pp.atlasPatch3dRangeD() == 1023);
            REQUIRE_FALSE(pp.asme_depth_occ_threshold_flag());
            REQUIRE(pp.atlasPatchDepthOccThreshold() == 0);
          }
        }

        AND_THEN("Set the atlas frame size in the ASPS") {
          for (const auto &atlas : renderFrame.atlas) {
            REQUIRE(atlas.asps.asps_frame_width() == atlas.attrFrame.getWidth());
            REQUIRE(atlas.asps.asps_frame_height() == atlas.attrFrame.getHeight());
          }
        }

        AND_THEN("The block to patch map uniformly points to patch 0") {
          for (const auto &atlas : renderFrame.atlas) {
            const auto ppbs = 1 << atlas.asps.asps_log2_patch_packing_block_size();
            REQUIRE(atlas.blockToPatchMap.getWidth() * ppbs == atlas.attrFrame.getWidth());
            REQUIRE(atlas.blockToPatchMap.getHeight() * ppbs == atlas.attrFrame.getHeight());
            REQUIRE(std::all_of(atlas.blockToPatchMap.getPlane(0).cbegin(),
                                atlas.blockToPatchMap.getPlane(0).cend(),
                                [](auto value) { return value == 0; }));
          }
        }

        AND_THEN("The occupancy map is uniform 1") {
          for (const auto &atlas : renderFrame.atlas) {
            REQUIRE(atlas.occFrame.getSize() == atlas.attrFrame.getSize());
            REQUIRE(std::all_of(atlas.occFrame.getPlane(0).cbegin(),
                                atlas.occFrame.getPlane(0).cend(),
                                [](auto value) { return value == 1; }));
          }
        }
      }
    }
  }

  SECTION("The depth is converted from 16-bit to 10-bit range prior to synthesis") {
    const auto inParams = sourceParams();
    auto inFrame = inputFrame(inParams);

    inFrame.front().depth.getPlane(0)[0] = 0xFFFF;
    inFrame.front().depth.getPlane(0)[1] = 0xFFF;
    inFrame.front().depth.getPlane(0)[2] = 0xFF;
    inFrame.front().depth.getPlane(0)[3] = 0xF;
    inFrame.front().depth.getPlane(0)[4] = 0;

    FakeSynthesizer::inspect_renderFrame_input =
        [](const TMIV::MivBitstream::AccessUnit &frame,
           const TMIV::MivBitstream::CameraConfig & /* viewportParams */) {
          REQUIRE(frame.atlas.front().geoFrame.getPlane(0)[0] == 0x3FF);
          REQUIRE(frame.atlas.front().geoFrame.getPlane(0)[1] == 0x40);
          REQUIRE(frame.atlas.front().geoFrame.getPlane(0)[2] == 0x4);
          REQUIRE(frame.atlas.front().geoFrame.getPlane(0)[3] == 0);
          REQUIRE(frame.atlas.front().geoFrame.getPlane(0)[4] == 0);
        };

    auto ssi = construct();
    auto params = ssi.optimizeParams(inParams);
    auto frame = ssi.optimizeFrame(inFrame);

    FakeSynthesizer::inspect_renderFrame_input = nullptr;
  }

  SECTION("Pass the depth low quality flag to the synthesizer") {
    const auto dlqf = GENERATE(false, true);

    FakeSynthesizer::inspect_renderFrame_input =
        [dlqf](const TMIV::MivBitstream::AccessUnit &frame,
               const TMIV::MivBitstream::CameraConfig & /* viewportParams */) {
          REQUIRE(frame.casps.has_value());
          REQUIRE(frame.casps->casps_miv_extension_present_flag());
          REQUIRE(frame.casps->casps_miv_extension().casme_depth_low_quality_flag() == dlqf);
        };

    auto ssi = construct();
    auto inParams = sourceParams();
    inParams.depthLowQualityFlag = dlqf;
    auto params = ssi.optimizeParams(inParams);
    auto frame = ssi.optimizeFrame(inputFrame(inParams));

    FakeSynthesizer::inspect_renderFrame_input = nullptr;
  }
}
