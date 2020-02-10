/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

#ifndef _TMIV_METADATA_VIEWINGSPACE_H_
#define _TMIV_METADATA_VIEWINGSPACE_H_

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Json.h>
#include <TMIV/Common/Vector.h>

#include <iosfwd>
#include <optional>
#include <variant>
#include <vector>

namespace TMIV::Metadata {
using Common::InputBitstream;
using Common::OutputBitstream;

struct PrimitiveShape;
struct ElementaryShape;

enum class PrimitiveShapeType {
  cuboid = 0,
  spheroid = 1,
  halfspace = 2
}; // In specification: primitive_shape_type[ e ]

enum class PrimitiveShapeOperation {
  add = 0,
  interpolate = 1
}; // In specification: primitive_shape_operation[ e ]

enum class ElementaryShapeOperation {
  add = 0,
  subtract = 1,
  intersect = 2
}; // In specification: elementary_shape_operation[ e ]

using ElementaryShapeVector = std::vector<std::pair<ElementaryShapeOperation, ElementaryShape>>;

// In specification: viewing_space( )
struct ViewingSpace {
  // In specification: num_elementary_shapes_minus1
  // In specification: elementary_shape_operation[ e ]
  // In specification: elementary_shape[ e ]
  ElementaryShapeVector elementaryShapes;

  friend std::ostream &operator<<(std::ostream &stream, const ViewingSpace &viewingSpace);
  bool operator==(const ViewingSpace &other) const;
  bool operator!=(const ViewingSpace &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &) -> ViewingSpace;
  void encodeTo(OutputBitstream &) const;

  static auto loadFromJson(const Common::Json &node) -> ViewingSpace;
};

using PrimitiveShapeVector = std::vector<PrimitiveShape>;

// In specification: elementary_shape( e )
struct ElementaryShape {
  // In specification: num_primitive_shapes_minus1
  // In specification: primitive_shape_type[ e ][ s ]
  PrimitiveShapeVector primitives;

  // In specification: primitive_shape_operation[ e ]
  PrimitiveShapeOperation primitiveOperation{};

  friend std::ostream &operator<<(std::ostream &stream, const ElementaryShape &shape);
  bool operator==(const ElementaryShape &other) const;
  bool operator!=(const ElementaryShape &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &) -> ElementaryShape;
  void encodeTo(OutputBitstream &) const;

  static auto loadFromJson(const Common::Json &node) -> ElementaryShape;
};

// In specification: cuboid_primitive( e, s )
struct Cuboid {
  // In specification: center_x[e][s]
  // In specification: center_y[e][s]
  // In specification: center_z[e][s]
  Common::Vec3f center{};
  // In specification: size_x[e][s]
  // In specification: size_y[e][s]
  // In specification: size_z[e][s]
  Common::Vec3f size{};

  friend std::ostream &operator<<(std::ostream &stream, const Cuboid &cuboid);
  bool operator==(const Cuboid &other) const;
  bool operator!=(const Cuboid &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &) -> Cuboid;
  void encodeTo(OutputBitstream &) const;

  static auto loadFromJson(const Common::Json &node) -> Cuboid;
};

// In specification: sphere_primitive( e, s )
struct Spheroid {
  // In specification: center_x[e][s]
  // In specification: center_y[e][s]
  // In specification: center_z[e][s]
  Common::Vec3f center{};
  // In specification: radius[e][s]
  Common::Vec3f radius{};

  friend std::ostream &operator<<(std::ostream &stream, const Spheroid &spheroid);
  bool operator==(const Spheroid &other) const;
  bool operator!=(const Spheroid &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &) -> Spheroid;
  void encodeTo(OutputBitstream &) const;

  static auto loadFromJson(const Common::Json &node) -> Spheroid;
};

// In specification: halfspace_primitive( e, s )
struct Halfspace {
  // In specification: normal_x[e][s]
  // In specification: normal_y[e][s]
  // In specification: normal_z[e][s]
  Common::Vec3f normal{};
  // In specification: distance[e][s]
  float distance{};

  friend std::ostream &operator<<(std::ostream &stream, const Halfspace &halfspace);
  bool operator==(const Halfspace &other) const;
  bool operator!=(const Halfspace &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &) -> Halfspace;
  void encodeTo(OutputBitstream &) const;

  static auto loadFromJson(const Common::Json &node) -> Halfspace;
};

struct PrimitiveShape {
  // In specification: primitive_shape_type( e, s )
  // In specification: cuboid_primitive( e, s )
  // In specification: spheroid_primitive( e, s )
  // In specification: halfspace_primitive( e, s )
  std::variant<Cuboid, Spheroid, Halfspace> primitive;

  auto shapeType() const -> PrimitiveShapeType;

  // In specification: guard_band_present_flag[ e ]
  // In specification: guard_band_size[ e ]
  std::optional<float> guardBandSize{};

  // In specification: primitive_orientation_present_flag[ e ]
  // In specification: primitive_shape_yaw[ e ]
  // In specification: primitive_shape_pitch[ e ]
  // In specification: primitive_shape_roll[ e ]
  std::optional<Common::Vec3f> rotation{};

  // In specification: viewing_direction_constraint_present_flag[ e ]
  // In specification: guard_band_present_flag[ e ]
  // In specification: guard_band_direction_size[ e ]
  // In specification: primitive_shape_viewing_direction_yaw_center[ e ]
  // In specification: primitive_shape_viewing_direction_yaw_range[ e ]
  // In specification: primitive_shape_viewing_direction_pitch_center[ e ]
  // In specification: primitive_shape_viewing_direction_pitch_range[ e ]
  struct ViewingDirectionConstraint {
    std::optional<float> guardBandDirectionSize{};
    float yawCenter{};
    float yawRange{360.f};
    float pitchCenter{};
    float pitchRange{180.f};

    bool operator==(const ViewingDirectionConstraint &other) const;
    bool operator!=(const ViewingDirectionConstraint &other) const { return !operator==(other); }
  };
  std::optional<ViewingDirectionConstraint> viewingDirectionConstraint;

  friend std::ostream &operator<<(std::ostream &stream, const PrimitiveShape &shape);
  bool operator==(const PrimitiveShape &other) const;
  bool operator!=(const PrimitiveShape &other) const { return !operator==(other); }

  static auto loadFromJson(const Common::Json &node) -> PrimitiveShape;
};

inline auto PrimitiveShape::shapeType() const -> PrimitiveShapeType {
  assert(primitive.index() != std::variant_npos);
  if (std::holds_alternative<Cuboid>(primitive))
    return PrimitiveShapeType::cuboid;
  if (std::holds_alternative<Spheroid>(primitive))
    return PrimitiveShapeType::spheroid;
  if (std::holds_alternative<Halfspace>(primitive))
    return PrimitiveShapeType::halfspace;
  abort();
}

} // namespace TMIV::Metadata

#endif
