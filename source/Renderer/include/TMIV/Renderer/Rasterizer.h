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

#ifndef TMIV_RENDERER_RASTERIZER_H
#define TMIV_RENDERER_RASTERIZER_H

#include "AccumulatingPixel.h"
#include "Projector.h"

namespace TMIV::Renderer {
using SceneVertexDescriptorList = std::vector<SceneVertexDescriptor>;
using TriangleDescriptorList = std::vector<TriangleDescriptor>;
using ImageVertexDescriptorList = std::vector<ImageVertexDescriptor>;

class Rasterizer {
public:
  using Exception = std::logic_error;

  // Construct a rasterizer with specified blender and a frame buffer of
  // specified size. The number of strips for concurrent processing is
  // chosen based on image size and hardware concurrency.
  Rasterizer(AccumulatingPixel pixel, Common::Vec2i size);

  // Construct a rasterizer with specified blender and a frame buffer of
  // specified size, and specify the number of strips for concurrent processing.
  Rasterizer(AccumulatingPixel pixel, Common::Vec2i size, int32_t numStrips);

  // Submit a batch of triangles
  //
  // The batch is stored within the Rasterizer for later processing.
  // Multiple batches may be submitted sequentially.
  void submit(const ImageVertexDescriptorList &vertices, std::vector<Common::Vec3f> colors,
              const TriangleDescriptorList &triangles);

  // Raster all submitted batches
  //
  // On return the output maps may be calculated.
  void run();

  // Output the depth map (in meters)
  //
  // For effiency normalized disparity is blended. Because triangles are
  // small there will be virtually no difference in practice. However when
  // depth interpolation has to be more accurate, it is possible to
  // specify the depth map as an attribute.
  [[nodiscard]] auto depth() const -> Common::Mat<float>;

  // Output the normalized disparity map (in diopters)
  //
  // For effiency normalized disparity is blended. Because triangles are
  // small there will be virtually no difference in practice. However when
  // depth interpolation has to be more accurate, it is possible to
  // specify the depth map as an attribute.
  [[nodiscard]] auto normDisp() const -> Common::Mat<float>;

  // Output the quality estimate (in a.u.)
  [[nodiscard]] auto normWeight() const -> Common::Mat<float>;

  // Output color map
  [[nodiscard]] auto color() const -> Common::Mat<Common::Vec3f> {
    Common::Mat<Common::Vec3f> matrix(m_size);
    auto i_matrix = std::begin(matrix);
    visit([&i_matrix](const PixelValue &x) {
      *i_matrix++ = x.color;
      return true;
    });
    POSTCONDITION(i_matrix == std::end(matrix));
    return matrix;
  }

  // Visit each pixel in row-major order
  //
  // Signature: bool(const PixelValue& value)
  // Return false to stop visiting
  template <class Visitor> void visit(Visitor visitor) const {
    PRECONDITION(m_batches.empty());

    for (const auto &strip : m_strips) {
      for (const auto &accumulator : strip.matrix) {
        if (!visitor(m_pixel.average(accumulator))) {
          return;
        }
      }
    }
  }

private:
  struct Strip {
    // Strip dimensions
    const int32_t i1{};
    const int32_t i2{};
    const int32_t cols{};

    [[nodiscard]] constexpr auto rows() const -> int32_t { return i2 - i1; }

    // Batches of triangles to be processed
    std::vector<TriangleDescriptorList> batches;

    // The strip of pixels with intermediate blending state
    std::vector<PixelAccumulator> matrix;
  };

  // Information of each batch that is shared between strips
  //
  // Note that m_batches.size() == m_strips[].batches.size()
  struct Batch {
    ImageVertexDescriptorList vertices;
    std::vector<Common::Vec3f> color;
  };

  using Size = Common::Mat<float>::tuple_type;

  void submitTriangle(TriangleDescriptor descriptor, const Batch &batch);
  void rasterTriangle(TriangleDescriptor descriptor, const Batch &batch, Strip &strip);
  void clearBatches();

  AccumulatingPixel m_pixel;
  Size m_size{};
  float m_dk_di{}; // i for row, j for column and k for strip index
  std::vector<Strip> m_strips;
  std::vector<Batch> m_batches;
};
} // namespace TMIV::Renderer

#endif
