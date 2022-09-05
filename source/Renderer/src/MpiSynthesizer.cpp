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

#include <TMIV/Renderer/MpiSynthesizer.h>

#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <TMIV/Renderer/MpiRasterizer.h>
#include <TMIV/Renderer/reprojectPoints.h>

#include <array>

namespace TMIV::Renderer {
const auto depthErrorEps = 1E-4F;

namespace {
auto getGatherCoordinates(const Common::Vec2f &p, const std::array<size_t, 2> &sz)
    -> std::array<Common::Vec2i, 4> {
  int32_t w_last = static_cast<int32_t>(sz[0]) - 1;
  int32_t h_last = static_cast<int32_t>(sz[1]) - 1;

  int32_t x0 = std::clamp(static_cast<int32_t>(std::floor(p.x() - 0.5F)), 0, w_last);
  int32_t y0 = std::clamp(static_cast<int32_t>(std::floor(p.y() - 0.5F)), 0, h_last);

  int32_t x1 = std::min(x0 + 1, w_last);
  int32_t y1 = std::min(y0 + 1, h_last);

  return {Common::Vec2i{y1, x0}, Common::Vec2i{y1, x1}, Common::Vec2i{y0, x1},
          Common::Vec2i{y0, x0}};
}

template <typename MAT>
auto textureGather(const MAT &m, const std::array<Common::Vec2i, 4> &gc)
    -> Common::stack::Vec4<typename MAT::value_type> {
  return {m(gc[0].x(), gc[0].y()), m(gc[1].x(), gc[1].y()), m(gc[2].x(), gc[2].y()),
          m(gc[3].x(), gc[3].y())};
}

template <typename T>
auto textureGather(const std::function<T(uint32_t, uint32_t)> &fun,
                   const std::array<Common::Vec2i, 4> &gc) -> Common::stack::Vec4<T> {
  return {fun(gc[0].x(), gc[0].y()), fun(gc[1].x(), gc[1].y()), fun(gc[2].x(), gc[2].y()),
          fun(gc[3].x(), gc[3].y())};
}

auto getPatchPackingWidth(const MivBitstream::PatchParams &patch) -> int32_t {
  return patch.isRotated() ? patch.atlasPatch3dSizeV() : patch.atlasPatch3dSizeU();
}

} // namespace

class MpiSynthesizer::Impl {
public:
  using PixelAttribute = Renderer::PixelAttributes<Common::Vec2f, float, uint32_t, uint32_t>;
  using MpiRasterizer = Renderer::MpiRasterizer<Common::Vec2f, float, uint32_t, uint32_t>;

private:
  float m_minAlpha = 0.20F;
  Common::Mat<Common::Vec3f> m_blendingColor;
  Common::Mat<float> m_blendingDepth;
  Common::Mat<float> m_blendingTransparency;
  std::vector<std::tuple<int32_t, int32_t, int32_t, float>>
      m_blockBuffer; //{atlasId, patchIdx(in atlas), blockId(in patch), depth}
  int32_t m_blockSize{};

public:
  explicit Impl(const Common::Json &componentNode) {
    m_minAlpha = componentNode.require("minAlpha").as<float>();
  }

  Impl(float minAlpha) { m_minAlpha = minAlpha; }

  static auto isTriangleDimensionNotTooBig(float x0, float x1, float x2, float W) -> bool {
    float xMin = std::min(x0, std::min(x1, x2));
    float xMax = std::max(x0, std::max(x1, x2));

    return (xMax - xMin) < 0.25F * W;
  }

  // Handling properly the seam for 360 degrees scenes
  static auto shouldRepeat(const MivBitstream::ViewParams &viewParams) -> bool {
    return viewParams.ci.ci_cam_type() == MivBitstream::CiCamType::equirectangular &&
           0.99F * 360.F < viewParams.ci.ci_erp_phi_max() - viewParams.ci.ci_erp_phi_min();
  }

  auto renderFrame(const MivBitstream::AccessUnit &frame,
                   const MivBitstream::CameraConfig &cameraConfig) -> Common::RendererFrame {
    // 0 - Check for block size consistency
    if (frame.foc == 0) {
      prepare(frame);
    }

    // 1 - Update block buffer when atlas is updated
    //
    // NOTE(FT): this line should be called only when needed, since it increases artificially the
    // rendering time  ==> to be changed once the information of new intraPeriod coming from
    // MivDecoder reaches MpiSynthesizer
    allocateBlockBuffer(frame);

    // 2- Update viewport buffer in case of viewport size change
    if (m_blendingColor.m() !=
            cameraConfig.viewParams.ci.ci_projection_plane_height_minus1() + size_t{1} ||
        m_blendingColor.n() !=
            cameraConfig.viewParams.ci.ci_projection_plane_width_minus1() + size_t{1}) {
      allocateViewportBuffer(cameraConfig.viewParams);
    }

    // 3- Expand atlas in float format (texture and transparency)
    std::vector<Common::Mat<Common::Vec3f>> atlasColor;
    std::vector<Common::Mat<float>> atlasTransparency;
    std::tie(atlasColor, atlasTransparency) = expandAtlas(frame);

    // 4 - Blending
    MpiRasterizer rasterizer{cameraConfig.viewParams.ci.projectionPlaneSize()};

    auto [vertices, attributes, triangles] = createMesh(frame, cameraConfig.viewParams);

    std::fill(m_blendingColor.begin(), m_blendingColor.end(), Common::Vec3f({0.F, 0.F, 0.F}));
    std::fill(m_blendingDepth.begin(), m_blendingDepth.end(), -1.F);
    std::fill(m_blendingTransparency.begin(), m_blendingTransparency.end(), 0.F);

    rasterizer.submit(vertices, attributes, triangles);
    rasterizer.run([this, &frame, &atlasColor, &atlasTransparency](
                       const ViewportPosition2D &viewport, const std::array<float, 3> &weights,
                       const std::array<PixelAttribute, 3> &pixelAttributes) {
      return fragmentShader(viewport, weights, pixelAttributes, frame, atlasColor,
                            atlasTransparency);
    });

    // 5 - Shading
    std::transform(m_blendingColor.begin(), m_blendingColor.end(), m_blendingTransparency.begin(),
                   m_blendingColor.begin(),
                   [this](const auto &blendedColor, auto blendedTransparency) {
                     return (m_minAlpha < blendedTransparency)
                                ? (blendedColor / blendedTransparency)
                                : Common::Vec3f(0.F);
                   });

    // 6 - Disparity
    std::transform(m_blendingDepth.begin(), m_blendingDepth.end(), m_blendingTransparency.begin(),
                   m_blendingDepth.begin(), [&](auto blendedDepth, auto blendedTransparency) {
                     return ((m_minAlpha < blendedTransparency) && isValidDepth(blendedDepth))
                                ? std::clamp(1.F / blendedDepth,
                                             cameraConfig.viewParams.dq.dq_norm_disp_low(),
                                             cameraConfig.viewParams.dq.dq_norm_disp_high())
                                : NAN;
                   });

    // 7 - Quantization
    const auto depthTransform =
        MivBitstream::DepthTransform{cameraConfig.viewParams.dq, cameraConfig.bitDepthGeometry};
    auto viewport =
        Common::RendererFrame{quantizeTexture(m_blendingColor, cameraConfig.bitDepthTexture),
                              depthTransform.quantizeNormDisp(m_blendingDepth, 1)};
    viewport.texture.fillInvalidWithNeutral(viewport.geometry);

    return viewport;
  }

private:
  void prepare(const MivBitstream::AccessUnit &frame) {
    LIMITATION(all_of(frame.atlas.begin(), frame.atlas.end(), [&frame](const auto &atlas) {
      return frame.atlas.front().asps.asps_log2_patch_packing_block_size() ==
             atlas.asps.asps_log2_patch_packing_block_size();
    }));
    m_blockSize = uint16_t{1} << frame.atlas.front().asps.asps_log2_patch_packing_block_size();
  }

  void allocateBlockBuffer(const MivBitstream::AccessUnit &frame) {
    // Get average depth for all patches of all atlases
    std::vector<float> patchAverageDepth;
    std::vector<int32_t> patchIdx;
    std::vector<int32_t> patchAtlasId;
    int32_t idx_atlas = 0;

    for (const auto &atlas : frame.atlas) {
      if (atlas.asps.asps_miv_extension_present_flag() &&
          atlas.asps.asps_miv_extension().asme_ancillary_atlas_flag()) {
        continue;
      }

      const auto &patchList = atlas.patchParamsList;
      int32_t idx_patch = 0;
      for (const auto &p : patchList) {
        auto bits = static_cast<uint32_t>(atlas.asps.asps_geometry_2d_bit_depth_minus1()) + 1U;

        const auto depthTransform = MivBitstream::DepthTransform{
            frame.viewParamsList[p.atlasPatchProjectionId()].dq, p, bits};

        float d = depthTransform.expandDepth(p.atlasPatch3dOffsetD());

        patchAverageDepth.push_back(d);
        patchAtlasId.push_back(idx_atlas);
        patchIdx.push_back(idx_patch);
        idx_patch++;
      }
      idx_atlas++;
    }

    // Sort patches by increasing depth
    std::vector<int32_t> patchOrderId(patchAverageDepth.size());
    std::iota(patchOrderId.begin(), patchOrderId.end(), 0);
    std::sort(patchOrderId.begin(), patchOrderId.end(), [&](uint32_t id1, uint32_t id2) {
      if (std::abs(patchAverageDepth[id1] - patchAverageDepth[id2]) < depthErrorEps) {
        return id2 < id1;
      }
      return patchAverageDepth[id1] < patchAverageDepth[id2];
    });

    // Construct ordered block buffer
    auto blockArea = Common::sqr(m_blockSize);

    m_blockBuffer.clear();
    for (int32_t idx : patchOrderId) {
      const auto &patch = frame.atlas[patchAtlasId[idx]].patchParamsList[patchIdx[idx]];
      int32_t patch_area = static_cast<int32_t>(patch.atlasPatch2dSizeX()) *
                           static_cast<int32_t>(patch.atlasPatch2dSizeY());
      auto nbBlock = static_cast<int32_t>(patch_area / blockArea);
      auto offsetId = m_blockBuffer.size();

      m_blockBuffer.resize(
          offsetId + static_cast<size_t>(nbBlock),
          std::make_tuple(patchAtlasId[idx], patchIdx[idx], 0, patchAverageDepth[idx]));

      for (auto blockId = 0; blockId < nbBlock; blockId++, offsetId++) {
        std::get<2>(m_blockBuffer[offsetId]) = blockId;
      }
    }
  }

  void allocateViewportBuffer(const MivBitstream::ViewParams &viewportParams) {
    const auto size = viewportParams.ci.projectionPlaneSize();

    m_blendingColor.resize(size.y(), size.x());
    m_blendingDepth.resize(size.y(), size.x());
    m_blendingTransparency.resize(size.y(), size.x());
  }

  static auto expandTransparency(const MivBitstream::AccessUnit &frame, int32_t frameIdx)
      -> Common::Mat<float> {
    auto atlasId = frame.vps.vps_atlas_id(frameIdx);
    const auto &atlas = frame.atlas[frameIdx];

    auto maxValue = 0.F;

    for (uint8_t attrIdx = 0;
         attrIdx <= frame.vps.attribute_information(atlasId).ai_attribute_count(); attrIdx++) {
      if (frame.vps.attribute_information(atlasId).ai_attribute_type_id(attrIdx) ==
          MivBitstream::AiAttributeTypeId::ATTR_TRANSPARENCY) {
        maxValue = static_cast<float>(
            (1U << static_cast<uint32_t>(frame.vps.attribute_information(atlasId)
                                             .ai_attribute_2d_bit_depth_minus1(attrIdx) +
                                         1)) -
            1U);
        break;
      }
    }

    POSTCONDITION(0.F < maxValue);

    const auto attrIdx =
        frame.vps.attrIdxOf(atlasId, MivBitstream::AiAttributeTypeId::ATTR_TRANSPARENCY);
    PRECONDITION(attrIdx.has_value());
    const auto &transparencyFrame = atlas.attrFrameNF[*attrIdx];

    Common::Mat<float> transparencyMap({static_cast<size_t>(transparencyFrame.getHeight()),
                                        static_cast<size_t>(transparencyFrame.getWidth())});
    const auto &transparencyPlane = transparencyFrame.getPlane(0);

    std::transform(transparencyPlane.begin(), transparencyPlane.end(), transparencyMap.begin(),
                   [&](auto q) { return static_cast<float>(q) / maxValue; });

    return transparencyMap;
  }

  static auto expandAtlas(const MivBitstream::AccessUnit &frame)
      -> std::tuple<std::vector<Common::Mat<Common::Vec3f>>, std::vector<Common::Mat<float>>> {
    std::vector<Common::Mat<Common::Vec3f>> atlasColor;
    std::vector<Common::Mat<float>> atlasTransparency;
    int32_t idx = 0;

    for (const auto &atlas : frame.atlas) {
      atlasColor.emplace_back(expandTexture(atlas.texFrame));
      atlasTransparency.emplace_back(expandTransparency(frame, idx));

      idx++;
    }

    return {atlasColor, atlasTransparency};
  }

  auto createMesh(const MivBitstream::AccessUnit &frame,
                  const MivBitstream::ViewParams &viewportParams)
      -> std::tuple<ImageVertexDescriptorList, MpiRasterizer::AttributeMaps,
                    TriangleDescriptorList> {
    ImageVertexDescriptorList vertices;
    MpiRasterizer::AttributeMaps attributes;
    TriangleDescriptorList triangles;

    const auto sourceHelperList = ProjectionHelperList{frame.viewParamsList};
    const auto targetHelper = ProjectionHelper{viewportParams};

    for (const auto &block : m_blockBuffer) {
      // Initialization
      const auto &[atlas_id, patch_id, block_id, d] = block;

      const MivBitstream::AtlasAccessUnit &atlas = frame.atlas[atlas_id];
      const MivBitstream::PatchParams &patch = atlas.patchParamsList[patch_id];
      const auto viewIdx = frame.viewParamsList.indexOf(patch.atlasPatchProjectionId());
      const auto viewParams = frame.viewParamsList[viewIdx];

      // Block corners
      auto blockPerRow = getPatchPackingWidth(patch) / m_blockSize;

      auto x_atlas_0 =
          static_cast<int32_t>(patch.atlasPatch2dPosX()) + (block_id % blockPerRow) * m_blockSize;
      auto y_atlas_0 =
          static_cast<int32_t>(patch.atlasPatch2dPosY()) + (block_id / blockPerRow) * m_blockSize;

      const auto tl = Common::Vec2i{x_atlas_0, y_atlas_0};
      const auto tr = Common::Vec2i{x_atlas_0, y_atlas_0 + m_blockSize};
      const auto br = Common::Vec2i{x_atlas_0 + m_blockSize, y_atlas_0 + m_blockSize};
      const auto bl = Common::Vec2i{x_atlas_0 + m_blockSize, y_atlas_0};

      // Populate the vertices/attributes
      for (auto p : {tl, tr, br, bl}) {
        auto uv = Common::Vec2f{Common::floatCast, patch.atlasToView(p)};

        // Handling correctly the seam for 360 degrees scenes
        if (shouldRepeat(viewParams)) {
          const int32_t offset = 2;

          if (uv.x() < offset) {
            uv.x() -= offset;
          }
        }

        // Reprojection in viewport
        auto P = sourceHelperList[viewIdx].doUnprojection(
            {static_cast<float>(uv.x()) + 0.5F, static_cast<float>(uv.y()) + 0.5F}, d);
        auto q = targetHelper.doProjection(P);

        // Viewport position
        vertices.push_back({q.first, 0.F, 0.F});

        // Attribute #0 -> Atlas position
        std::get<0>(attributes).push_back(Common::Vec2f{Common::floatCast, p} + 0.5F);

        // Attribute #1 -> Viewport depth
        std::get<1>(attributes).push_back(q.second);

        // Attribute #2 -> Patch ID
        std::get<2>(attributes).emplace_back(static_cast<uint32_t>(patch_id));

        // Attribute #3 -> Atlas ID
        std::get<3>(attributes).emplace_back(static_cast<uint32_t>(atlas_id));
      }

      // Populate the triangles if not too big
      auto N = static_cast<int32_t>(vertices.size()) - 1;

      if (isTriangleDimensionNotTooBig(vertices[N - 3].position.x(), vertices[N - 2].position.x(),
                                       vertices[N - 1].position.x(),
                                       viewportParams.ci.projectionPlaneSizeF().x())) {
        triangles.push_back({{N - 3, N - 2, N - 1}, 0.F});
      }

      if (isTriangleDimensionNotTooBig(vertices[N - 3].position.x(), vertices[N - 1].position.x(),
                                       vertices[N].position.x(),
                                       viewportParams.ci.projectionPlaneSizeF().x())) {
        triangles.push_back({{N - 3, N - 1, N}, 0.F});
      }
    }

    return {vertices, attributes, triangles};
  }

  void fragmentShader(const ViewportPosition2D &InViewport, const std::array<float, 3> &weights,
                      const std::array<PixelAttribute, 3> &pixelAttributes,
                      const MivBitstream::AccessUnit &frame,
                      const std::vector<Common::Mat<Common::Vec3f>> &atlasColor,
                      const std::vector<Common::Mat<float>> &atlasTransparency) {
    auto &pixelAlpha = m_blendingTransparency(InViewport.y, InViewport.x);

    if (pixelAlpha < 1.F) {
      auto patchIdx = static_cast<uint16_t>(std::get<2>(pixelAttributes[0]));
      auto atlasId = std::get<3>(pixelAttributes[0]);

      auto texCoord = weights[0] * std::get<0>(pixelAttributes[0]) +
                      weights[1] * std::get<0>(pixelAttributes[1]) +
                      weights[2] * std::get<0>(pixelAttributes[2]);
      auto gatherCoord = getGatherCoordinates(texCoord, atlasColor[atlasId].sizes());

      auto depth = weights[0] * std::get<1>(pixelAttributes[0]) +
                   weights[1] * std::get<1>(pixelAttributes[1]) +
                   weights[2] * std::get<1>(pixelAttributes[2]);
      auto patchIdGathered = textureGather<uint16_t>(
          [&](uint32_t row, uint32_t col) {
            return frame.atlas[atlasId].filteredPatchIdx(row, col);
          },
          gatherCoord);
      auto colorGathered = textureGather(atlasColor[atlasId], gatherCoord);
      auto transparencyGathered = textureGather(atlasTransparency[atlasId], gatherCoord);

      Common::Vec2f q = {texCoord.x() - 0.5F, texCoord.y() - 0.5F};
      Common::Vec2f f = {q.x() - std::floor(q.x()), q.y() - std::floor(q.y())};
      Common::Vec2f fb = {1.F - f.x(), 1.F - f.y()};

      Common::Vec4f wInterp{fb.x() * f.y(), f.x() * f.y(), f.x() * fb.y(), fb.x() * fb.y()};

      Common::Vec3f oColor{};
      float oWeight = 0;

      for (uint32_t j = 0; j < 4U; j++) {
        if (patchIdx == patchIdGathered[j]) {
          oColor += wInterp[j] * transparencyGathered[j] * colorGathered[j];
          oWeight += wInterp[j] * transparencyGathered[j];
        }
      }

      if (0 < oWeight) {
        oColor = oColor / oWeight;
      }

      // Alpha saturate blending
      oWeight -= std::max(0.F, (pixelAlpha + oWeight - 1.F));

      if (0.F < oWeight) {
        auto &pixelColor = m_blendingColor(InViewport.y, InViewport.x);

        pixelColor += oWeight * oColor;
        pixelAlpha += oWeight;
      }

      // Min depth blending
      if (0.F < oWeight) {
        auto &pixelDepth = m_blendingDepth(InViewport.y, InViewport.x);
        pixelDepth = (pixelDepth < 0.F) ? depth : std::min(pixelDepth, depth);
      }
    }
  }
}; // namespace TMIV::Renderer

MpiSynthesizer::MpiSynthesizer(const Common::Json & /*rootNode*/, const Common::Json &componentNode)
    : m_impl(new Impl(componentNode)) {}

MpiSynthesizer::MpiSynthesizer(float minAlpha) : m_impl(new Impl(minAlpha)) {}

MpiSynthesizer::~MpiSynthesizer() = default;

auto MpiSynthesizer::renderFrame(const MivBitstream::AccessUnit &frame,
                                 const MivBitstream::CameraConfig &cameraConfig) const
    -> Common::RendererFrame {
  return m_impl->renderFrame(frame, cameraConfig);
}
} // namespace TMIV::Renderer
