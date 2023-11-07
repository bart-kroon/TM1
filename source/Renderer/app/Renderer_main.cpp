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

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/DepthQualityAssessor/IDepthQualityAssessor.h>
#include <TMIV/IO/IO.h>
#include <TMIV/MivBitstream/Formatters.h>
#include <TMIV/MivBitstream/SequenceConfig.h>
#include <TMIV/Renderer/Front/MultipleFrameRenderer.h>
#include <TMIV/Renderer/Front/mapInputToOutputFrames.h>

#include <filesystem>
#include <fstream>

using namespace std::string_view_literals;

namespace TMIV::Renderer {
void registerComponents();

namespace {
void touchKeys(const Common::Json &config) {
  IO::touchLoadSequenceConfigKeys(config);
  IO::touchLoadMultiviewFrameKeys(config);
  IO::touchLoadViewportMetadataKeys(config);
  IO::touchSaveViewportKeys(config);
}
} // namespace

class RenderApplication : public Common::Application {
private:
  IO::Placeholders m_placeholders;
  std::unique_ptr<DepthQualityAssessor::IDepthQualityAssessor> m_assessor;
  Renderer::Front::MultipleFrameRenderer m_renderer;
  std::multimap<int32_t, int32_t> m_inputToOutputFrameIdMap;
  std::optional<bool> m_depthLowQualityFlag;
  MivBitstream::SequenceConfig m_inputSequenceConfig;

public:
  explicit RenderApplication(std::vector<const char *> argv)
      : Common::Application{"Renderer",
                            std::move(argv),
                            Common::Application::Options{
                                {"-s", "Content ID (e.g. B for Museum)", false},
                                {"-n", "Number of input frames (e.g. 97)", false},
                                {"-N", "Number of output frames (e.g. 300)", false},
                                {"-f", "Input start frame (e.g. 23)", false},
                                {"-r", "Test point (e.g. QP3 or R0)", false},
                                {"-v", "Source view to render (e.g. v11)", true},
                                {"-P", "Pose trace to render (e.g. p02)", true}},
                            {}}
      , m_placeholders{optionValues("-s").front(), optionValues("-r").front(),
                       std::stoi(optionValues("-n"sv).front()),
                       std::stoi(optionValues("-N"sv).front()),
                       std::stoi(optionValues("-f"sv).front())}
      , m_assessor{Common::create<DepthQualityAssessor::IDepthQualityAssessor>(
            "DepthQualityAssessor", json(), json())}
      , m_renderer{json(), optionValues("-v"), optionValues("-P"), m_placeholders}
      , m_inputToOutputFrameIdMap{Renderer::Front::mapInputToOutputFrames(
            m_placeholders.numberOfInputFrames, m_placeholders.numberOfOutputFrames)} {
    if (const auto &node = json().optional("depthLowQualityFlag")) {
      m_depthLowQualityFlag = node.as<bool>();
    }
    touchKeys(json());
  }

  void run() override {
    json().checkForUnusedKeys();

    for (int32_t frameIdx = 0;; ++frameIdx) {
      // Check which frames to render if we would
      const auto range = m_inputToOutputFrameIdMap.equal_range(frameIdx);
      if (range.first == range.second) {
        return;
      }

      updateParams(frameIdx);
      const auto frame =
          IO::loadMultiviewFrame(json(), m_placeholders, m_inputSequenceConfig, frameIdx);
      const auto inputViewParamsList = m_inputSequenceConfig.sourceViewParams();

      if (!m_depthLowQualityFlag) {
        m_depthLowQualityFlag = m_assessor->isLowDepthQuality(inputViewParamsList, frame);
      }

      // Make up an access unit
      const auto au = accessUnit(frame, inputViewParamsList, frameIdx);

      m_renderer.renderMultipleFrames(au, range.first, range.second);
    }
  }

private:
  void updateParams(int32_t frameIdx) {
    if (frameIdx == 0) {
      m_inputSequenceConfig = IO::loadSequenceConfig(json(), m_placeholders, 0);
    } else if (auto sc = IO::tryLoadSequenceConfig(json(), m_placeholders, frameIdx)) {
      Common::logInfo("Updating parameters at frame {}", frameIdx);
      m_inputSequenceConfig = *sc;
    }
  }

  [[nodiscard]] auto accessUnit(const Common::DeepFrameList &frame,
                                const MivBitstream::ViewParamsList &vpl, int32_t frameIdx) const
      -> MivBitstream::AccessUnit {
    auto au = MivBitstream::AccessUnit{};
    au.viewParamsList = vpl;
    au.frameIdx = frameIdx;
    au.foc = frameIdx;

    PRECONDITION(m_depthLowQualityFlag.has_value());
    au.casps.emplace(MivBitstream::CommonAtlasSequenceParameterSetRBSP{});
    au.casps->casps_miv_extension().casme_depth_low_quality_flag(*m_depthLowQualityFlag);

    std::transform(
        frame.cbegin(), frame.cend(), std::back_inserter(au.atlas),
        [&vpl = au.viewParamsList, viewIdx = uint16_t{}](const Common::DeepFrame &frame_) mutable {
          return atlasAccessUnit(frame_, vpl[viewIdx++].viewId);
        });
    return au;
  }

  [[nodiscard]] static auto atlasAccessUnit(const Common::DeepFrame &frame,
                                            MivBitstream::ViewId viewId)
      -> MivBitstream::AtlasAccessUnit {
    auto aau = MivBitstream::AtlasAccessUnit();

    const auto w = frame.texture.getWidth();
    const auto h = frame.texture.getHeight();

    aau.asps.asps_frame_width(static_cast<uint16_t>(w));
    aau.asps.asps_frame_height(static_cast<uint16_t>(h));
    aau.asps.asps_miv_extension().asme_max_entity_id(0);

    aau.texFrame = Common::yuv444(frame.texture);

    aau.geoFrame.createY({w, h}, 10);
    const auto maxInValue = Common::maxLevel(frame.geometry.getBitDepth());
    const auto maxOutValue = Common::maxLevel(aau.geoFrame.getBitDepth());
    std::transform(frame.geometry.getPlane(0).cbegin(), frame.geometry.getPlane(0).cend(),
                   aau.geoFrame.getPlane(0).begin(), [=](uint16_t value) {
                     return static_cast<uint16_t>((value * maxOutValue + maxInValue / 2) /
                                                  maxInValue);
                   });

    aau.occFrame.createY({w, h});
    aau.occFrame.fillOne();

    auto &pp = aau.patchParamsList.emplace_back();
    pp.atlasPatchProjectionId(viewId);
    pp.atlasPatchOrientationIndex(MivBitstream::FlexiblePatchOrientation::FPO_NULL);
    pp.atlasPatch2dSizeX(frame.texture.getWidth());
    pp.atlasPatch2dSizeY(frame.texture.getHeight());
    pp.atlasPatch3dRangeD(Common::maxLevel(aau.geoFrame.getBitDepth()));

    const auto ppbs = std::gcd(128, std::gcd(w, h));
    aau.asps.asps_log2_patch_packing_block_size(Common::ceilLog2(ppbs));
    aau.blockToPatchMap.createY({w / ppbs, h / ppbs}, 16);
    std::fill(aau.blockToPatchMap.getPlane(0).begin(), aau.blockToPatchMap.getPlane(0).end(),
              uint16_t{});

    return aau;
  }
};
} // namespace TMIV::Renderer

auto main(int argc, char *argv[]) -> int32_t {
  try {
    TMIV::Renderer::registerComponents();
    TMIV::Renderer::RenderApplication app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (...) {
    return TMIV::Common::handleException();
  }
}
