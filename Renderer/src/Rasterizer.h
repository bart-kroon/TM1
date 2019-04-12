/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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

#ifndef _TMIV_RENDERER_RASTERIZER_H_
#define _TMIV_RENDERER_RASTERIZER_H_

#include "AccumulatingPixel.h"
#include "Engine.h"

namespace TMIV::Renderer {
template <typename... T> class Rasterizer {
public:
  using Exception = std::logic_error;
  using Pixel = AccumulatingPixel<T...>;
  using Attributes = PixelAttributes<T...>;
  using Accumulator = PixelAccumulator<T...>;
  using Value = PixelValue<T...>;
  using AttributeMaps = std::tuple<std::vector<T>...>;

  // Construct a rasterizer with specified blender and a frame buffer of
  // specified size
  Rasterizer(Pixel pixel, Common::Vec2i size);

  // Submit a batch of triangles
  //
  // The batch is stored within the Rasterizer for later processing.
  // Multiple batches may be submitted sequentially.
  void submit(ImageVertexDescriptorList vertices, AttributeMaps attributes,
              const TriangleDescriptorList &triangles);

  // Raster all submitted batches
  //
  // On return the output maps may be calculated.
  void run();

  // Output the depth map (in meters)
  auto depth() const -> Common::Mat<float>;

  // Output the normalized disparity map (in diopters)
  auto normDisp() const -> Common::Mat<float>;

  // Output the quality estimate (in a.u.)
  auto normWeight() const -> Common::Mat<float>;

  // Output attribute map I (e.g. color)
  template <size_t I>
  auto attribute() const -> Common::Mat<std::tuple_element_t<I, Attributes>>;

  // Visit each pixel in row-major order
  //
  // Signature: bool(const Value& value)
  // Return false to stop visiting
  template <class Visitor> void visit(Visitor visitor) const;

private:
  struct Strip {
    // Strip dimensions
    const int i1;
    const int i2;
    const int cols;

    // Batches of triangles to be processed
    std::vector<TriangleDescriptorList> batches;

    // The strip of pixels with intermediate blending state
    Common::Mat<Accumulator> matrix;
  };

  // Information of each batch that is shared between strips
  //
  // Note that m_batches.size() == m_strips[].batches.size()
  struct Batch {
    const ImageVertexDescriptorList vertices;
    const AttributeMaps attributes;
  };

  using Size = Common::Mat<float>::tuple_type;

  void submitTriangle(TriangleDescriptor descriptor, const Batch &batch);
  void rasterTriangle(TriangleDescriptor descriptor, const Strip &strip,
                      const Batch &batch);
  void clearBatches();

  template <size_t... I>
  static auto fetchAttributes(int index, const AttributeMaps &attributes,
                              std::index_sequence<I...>) -> Attributes {
    return {std::get<I>(attributes)[index]...};
  }

  template <size_t... I>
  static auto interpolateAttributes(float w0, const Attributes &a0, float w1,
                                    const Attributes &a1, float w2,
                                    const Attributes &a2,
                                    std::index_sequence<I...>) -> Attributes {
    return {std::tuple_element_t<I, Attributes>{
        w0 * std::get<I>(a0) + w1 * std::get<I>(a1) + w2 * std::get<I>(a2)}...};
  }

  const Pixel m_pixel;
  const Size m_size;
  float m_dk_di; // i for row, j for column and k for strip index
  std::vector<Strip> m_strips;
  std::vector<Batch> m_batches;
};
} // namespace TMIV::Renderer

#include "Rasterizer.hpp"

#endif