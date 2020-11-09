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

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/DepthQualityAssessor/IDepthQualityAssessor.h>
#include <TMIV/IO/IO.h>
#include <TMIV/MivBitstream/SequenceConfig.h>
#include <TMIV/Renderer/IRenderer.h>
#include <TMIV/Renderer/mapInputToOutputFrames.h>

#include <filesystem>
#include <iostream>

namespace TMIV::Renderer {
void registerComponents();

class RenderApplication : public Common::Application {
private:
  std::unique_ptr<IRenderer> m_renderer;
  std::multimap<int, int> m_inputToOutputFrameIdMap;
  MivBitstream::SequenceConfig m_params;
  std::optional<bool> m_depthLowQualityFlag;

public:
  explicit RenderApplication(std::vector<const char *> argv)
      : Common::Application{"Renderer", std::move(argv)}
      , m_renderer{create<IRenderer>("Renderer")}
      , m_inputToOutputFrameIdMap{mapInputToOutputFrames(json())} {
    if (const auto &node = json().optional("depthLowQualityFlag")) {
      m_depthLowQualityFlag = node.as<bool>();
    }
  }

  void run() override {
    for (std::int32_t foc = 0;; ++foc) {
      // Check which frames to render if we would
      const auto range = m_inputToOutputFrameIdMap.equal_range(foc);
      if (range.first == range.second) {
        return;
      }

      updateParams(foc);
      const auto vpl = m_params.sourceViewParams();
      const auto frame = IO::loadSourceFrame(json(), vpl.viewSizes(), vpl.viewNames(), foc);

      if (!m_depthLowQualityFlag) {
        m_depthLowQualityFlag = isDepthLowQuality(frame, vpl);
      }

      // Make up an access unit
      const auto au = accessUnit(frame, vpl, foc);

      // Render multiple frames
      for (auto i = range.first; i != range.second; ++i) {
        renderFrame(au, i->second);
      }
    }
  }

private:
  void updateParams(std::int32_t foc) {
    const auto path =
        IO::getFullPath(json(), "SourceDirectory", "SourceSequenceConfigPathFmt", foc);
    if (std::filesystem::exists(path)) {
      std::cout << "Updating parameters at frame " << foc << '\n';
      std::ifstream stream{path};
      const auto json = Common::Json::loadFrom(stream);
      m_params = MivBitstream::SequenceConfig{json};
    }
  }

  void renderFrame(const Decoder::AccessUnit &frame, int outputFrameId) {
    std::cout << "Rendering input frame " << frame.foc << " to output frame " << outputFrameId
              << ", with target viewport:\n";
    const auto viewportParams = IO::loadViewportMetadata(json(), outputFrameId);
    viewportParams.printTo(std::cout, 0);

    const auto viewport = m_renderer->renderFrame(frame, viewportParams);
    IO::saveViewport(json(), outputFrameId, {yuv420p(viewport.first), viewport.second});
  }

  // TODO(BK): Add a IRenderer::renderFrame overload that takes a MVD16Frame
  [[nodiscard]] auto accessUnit(const Common::MVD16Frame &frame,
                                const MivBitstream::ViewParamsList &vpl, std::int32_t foc) const
      -> Decoder::AccessUnit {
    auto au = Decoder::AccessUnit{};
    au.viewParamsList = vpl;
    au.foc = foc;

    assert(m_depthLowQualityFlag.has_value());
    au.vps.vps_extension_present_flag(true)
        .vps_miv_extension_present_flag(true)
        .vps_miv_extension()
        .vme_depth_low_quality_flag(*m_depthLowQualityFlag);

    std::transform(frame.cbegin(), frame.cend(), std::back_inserter(au.atlas),
                   [viewIndex = uint16_t{}](const Common::TextureDepth16Frame &frame) mutable {
                     return atlasAccessUnit(frame, viewIndex++);
                   });
    return au;
  }

  [[nodiscard]] static auto atlasAccessUnit(const Common::TextureDepth16Frame &frame,
                                            std::uint16_t viewIndex) -> Decoder::AtlasAccessUnit {
    auto aau = Decoder::AtlasAccessUnit();

    const auto w = frame.texture.getWidth();
    const auto h = frame.texture.getHeight();

    aau.asps.asps_frame_width(static_cast<uint16_t>(w));
    aau.asps.asps_frame_height(static_cast<uint16_t>(h));

    aau.attrFrame = Common::yuv444p(frame.texture);

    aau.geoFrame.resize(w, h);
    std::transform(frame.depth.getPlane(0).cbegin(), frame.depth.getPlane(0).cend(),
                   aau.geoFrame.getPlane(0).begin(), [](uint16_t value) {
                     return static_cast<uint16_t>((value * 0x3FF + 0x7FFF) / 0xFFFF);
                   });

    aau.occFrame.resize(w, h);
    aau.occFrame.fillOne();

    auto &pp = aau.patchParamsList.emplace_back();
    pp.atlasPatchProjectionId(viewIndex);
    pp.atlasPatchOrientationIndex(MivBitstream::FlexiblePatchOrientation::FPO_NULL);
    pp.atlasPatch2dSizeX(frame.texture.getWidth());
    pp.atlasPatch2dSizeY(frame.texture.getHeight());

    const auto ppbs = std::gcd(128, std::gcd(w, h));
    aau.asps.asps_log2_patch_packing_block_size(Common::ceilLog2(ppbs));
    aau.blockToPatchMap.resize(w / ppbs, h / ppbs);
    std::fill(aau.blockToPatchMap.getPlane(0).begin(), aau.blockToPatchMap.getPlane(0).end(),
              uint16_t{});

    return aau;
  }

  auto isDepthLowQuality(const Common::MVD16Frame &frame, const MivBitstream::ViewParamsList &vpl)
      -> bool {
    const auto assessor =
        create<DepthQualityAssessor::IDepthQualityAssessor>("DepthQualityAssessor");
    return assessor->isLowDepthQuality(vpl, frame);
  }
};
} // namespace TMIV::Renderer

auto main(int argc, char *argv[]) -> int {
  try {
    TMIV::Renderer::registerComponents();
    TMIV::Renderer::RenderApplication app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (std::runtime_error &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  } catch (std::logic_error &e) {
    std::cerr << e.what() << std::endl;
    return 3;
  }
}
