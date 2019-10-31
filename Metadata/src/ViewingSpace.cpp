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

#include <TMIV/Metadata/ViewingSpace.h>

#include <TMIV/Metadata/Bitstream.h>

#include <TMIV/Common/Half.h>

#include <cassert>

using namespace std;

namespace TMIV::Metadata {

using Common::Half;

auto inline decodeHalf(InputBitstream &stream) -> float { return Half::decode(stream.getUint16()); }
inline void encodeHalf(const float x, OutputBitstream &stream) {
  stream.putUint16(Half(x).encode());
}

auto operator<<(std::ostream &stream, const ViewingSpace &viewingSpace) -> std::ostream & {
  stream << "Viewing space:" << endl;
  for (const auto &s : viewingSpace.elementaryShapes) {
    stream << (s.first == ElementaryShapeOperation::add ? "add " : "subtract ");
    stream << '(' << s.second << ')' << endl;
  }
  return stream;
}

auto ViewingSpace::operator==(const ViewingSpace &other) const -> bool {
  return (elementaryShapes == other.elementaryShapes);
}

auto ViewingSpace::decodeFrom(InputBitstream &stream) -> ViewingSpace {
  ViewingSpace vs;
  size_t numShapes = stream.getUExpGolomb() + 1;
  vs.elementaryShapes.reserve(numShapes);
  for (auto i = 0; i < numShapes; ++i) {
    const auto op = ElementaryShapeOperation(stream.readBits(1));
    const auto shape = ElementaryShape::decodeFrom(stream);
    vs.elementaryShapes.emplace_back(op, shape);
  }
  return vs;
}

void ViewingSpace::encodeTo(OutputBitstream &stream) const {
  assert(!elementaryShapes.empty());
  stream.putUExpGolomb(elementaryShapes.size() - 1);
  for (const auto& shape : elementaryShapes) {
    stream.writeBits(uint_least64_t(shape.first), 1);
    shape.second.encodeTo(stream);
  }
}

auto operator<<(std::ostream &stream, const ElementaryShape &elementaryShape) -> std::ostream & {
  stream << (elementaryShape.primitiveOperation == PrimitiveShapeOperation::interpolate
                 ? "interpolate"
                 : "add");
  for (const auto &p : elementaryShape.primitives) {
	stream << ", ";
    stream << p;
  }
  return stream;
}

auto ElementaryShape::operator==(const ElementaryShape &other) const -> bool {
  return (primitives == other.primitives);
}

auto ElementaryShape::decodeFrom(InputBitstream &stream) -> ElementaryShape {
  ElementaryShape elementaryShape;
  const auto numPrimitives = stream.readBits(8) + 1;
  elementaryShape.primitiveOperation = PrimitiveShapeOperation(stream.readBits(1));
  const auto guardBandPresent = stream.getFlag();
  const auto orientationPresent = stream.getFlag();
  const auto directionConstraintPresent = stream.getFlag();
  elementaryShape.primitives.reserve(numPrimitives);
  for (auto i = 0; i < numPrimitives; ++i) {
    PrimitiveShape primitiveShape;
    const auto shapeType = PrimitiveShapeType(stream.readBits(2));
    switch (shapeType) {
    case PrimitiveShapeType::cuboid:
      primitiveShape.primitive = Cuboid::decodeFrom(stream);
      break;
    case PrimitiveShapeType::spheroid:
      primitiveShape.primitive = Spheroid::decodeFrom(stream);
      break;
    case PrimitiveShapeType::halfspace:
      primitiveShape.primitive = Halfspace::decodeFrom(stream);
      break;
    default:
      abort();
    }
    if (guardBandPresent) {
      primitiveShape.guardBandSize = decodeHalf(stream);
    }
    if (orientationPresent) {
      primitiveShape.rotation = Common::Vec3f();
      primitiveShape.rotation.value().x() = decodeHalf(stream);
      primitiveShape.rotation.value().y() = decodeHalf(stream);
      primitiveShape.rotation.value().z() = decodeHalf(stream);
    }
    if (directionConstraintPresent) {
      auto vdc = PrimitiveShape::ViewingDirectionConstraint();
      if (guardBandPresent) {
        vdc.guardBandDirectionSize = decodeHalf(stream);
      }
      vdc.yawCenter = decodeHalf(stream);
      vdc.yawRange = decodeHalf(stream);
      vdc.pitchCenter = decodeHalf(stream);
      vdc.pitchRange = decodeHalf(stream);
      primitiveShape.viewingDirectionConstraint = vdc;
    }
    elementaryShape.primitives.emplace_back(primitiveShape);
  }
  return elementaryShape;
}

void ElementaryShape::encodeTo(OutputBitstream &stream) const {
  assert(!primitives.empty());
  bool guardBandPresent{};
  bool orientationPresent{};
  bool directionConstraintPresent{};
  for (const auto &p : primitives) {
    guardBandPresent |= p.guardBandSize.has_value();
    orientationPresent |= p.rotation.has_value();
    if (p.viewingDirectionConstraint.has_value()) {
      directionConstraintPresent |= true;
      guardBandPresent |= p.viewingDirectionConstraint.value().guardBandDirectionSize.has_value();
    }
  }
  stream.writeBits(primitives.size() - 1, 8);
  stream.writeBits(unsigned(primitiveOperation), 1);
  stream.putFlag(guardBandPresent);
  stream.putFlag(orientationPresent);
  stream.putFlag(directionConstraintPresent);
  for (const auto &p : primitives) {
    stream.writeBits(unsigned(p.shapeType()), 2);
    visit([&](const auto &x) { x.encodeTo(stream); }, p.primitive);
    if (guardBandPresent) {
      encodeHalf(p.guardBandSize.value_or(0.F), stream);
    }
    if (orientationPresent) {
      const Common::Vec3f r = p.rotation.value_or(Common::Vec3f());
      encodeHalf(r.x(), stream);
      encodeHalf(r.y(), stream);
      encodeHalf(r.z(), stream);
    }
    if (directionConstraintPresent) {
      const auto vdc =
          p.viewingDirectionConstraint.value_or(PrimitiveShape::ViewingDirectionConstraint());
      if (guardBandPresent) {
        encodeHalf(vdc.guardBandDirectionSize.value_or(0.F), stream);
      }
      encodeHalf(vdc.yawCenter, stream);
      encodeHalf(vdc.yawRange, stream);
      encodeHalf(vdc.pitchCenter, stream);
      encodeHalf(vdc.pitchRange, stream);
    }
  }
}

auto operator<<(std::ostream &stream, const PrimitiveShape &shape) -> std::ostream & {
  visit([&](const auto &x) { stream << x; }, shape.primitive);
  if (shape.guardBandSize.has_value()) {
    stream << " guardband " << shape.guardBandSize.value();
  }
  if (shape.rotation.has_value()) {
    stream << " rotation " << shape.rotation.value();
  }
  if (shape.viewingDirectionConstraint.has_value()) {
    const auto &vdc = shape.viewingDirectionConstraint.value();
    stream << " yaw " << vdc.yawCenter << "+/-" << 0.5F * vdc.yawRange << " pitch "
           << vdc.pitchRange << "+/-" << 0.5F * vdc.pitchRange;
    if (vdc.guardBandDirectionSize.has_value()) {
      stream << " guardband " << vdc.guardBandDirectionSize.value();
    }
  }
  return stream;
}

auto PrimitiveShape::operator==(const PrimitiveShape &other) const -> bool {
  if (primitive != other.primitive) {
    return false;
  }
  if (guardBandSize != other.guardBandSize) {
    return false;
  }
  if (rotation != other.rotation) {
    return false;
  }
  if (viewingDirectionConstraint != other.viewingDirectionConstraint) {
    return false;
  }
  return true;
}

auto PrimitiveShape::ViewingDirectionConstraint::
operator==(const ViewingDirectionConstraint& other) const -> bool {
  if (guardBandDirectionSize != other.guardBandDirectionSize) {
    return false;
  }
  if (yawCenter != other.yawCenter || pitchCenter != other.pitchCenter) {
    return false;
  }
  if (yawRange != other.yawRange || pitchRange != other.pitchRange) {
    return false;
  }
  return true;
}

auto operator<<(std::ostream &stream, const Cuboid &cuboid) -> std::ostream & {
  stream << "cuboid " << cuboid.center << " size " << cuboid.size;
  return stream;
}

auto Cuboid::operator==(const Cuboid &other) const -> bool {
  return center == other.center && size == other.size;
}

auto Cuboid::decodeFrom(InputBitstream &stream) -> Cuboid {
  Cuboid cuboid;
  cuboid.center.x() = decodeHalf(stream);
  cuboid.center.y() = decodeHalf(stream);
  cuboid.center.z() = decodeHalf(stream);
  cuboid.size.x() = decodeHalf(stream);
  cuboid.size.y() = decodeHalf(stream);
  cuboid.size.z() = decodeHalf(stream);
  return cuboid;
}

void Cuboid::encodeTo(OutputBitstream &stream) const {
  encodeHalf(center.x(), stream);
  encodeHalf(center.y(), stream);
  encodeHalf(center.z(), stream);
  encodeHalf(size.x(), stream);
  encodeHalf(size.y(), stream);
  encodeHalf(size.z(), stream);
}

auto operator<<(std::ostream &stream, const Spheroid &spheroid) -> std::ostream & {
  stream << "spheroid " << spheroid.center << " radius " << spheroid.radius;
  return stream;
}

auto Spheroid::operator==(const Spheroid &other) const -> bool {
  return center == other.center && radius == other.radius;
}

auto Spheroid::decodeFrom(InputBitstream &stream) -> Spheroid {
  Spheroid spheroid;
  spheroid.center.x() = decodeHalf(stream);
  spheroid.center.y() = decodeHalf(stream);
  spheroid.center.z() = decodeHalf(stream);
  spheroid.radius.x() = decodeHalf(stream);
  spheroid.radius.y() = decodeHalf(stream);
  spheroid.radius.z() = decodeHalf(stream);
  return spheroid;
}

void Spheroid::encodeTo(OutputBitstream &stream) const {
  encodeHalf(center.x(), stream);
  encodeHalf(center.y(), stream);
  encodeHalf(center.z(), stream);
  encodeHalf(radius.x(), stream);
  encodeHalf(radius.y(), stream);
  encodeHalf(radius.z(), stream);
}

auto operator<<(std::ostream &stream, const Halfspace &halfspace) -> std::ostream & {
  stream << "halfspace " << halfspace.normal << " distance " << halfspace.distance;
  return stream;
}

auto Halfspace::operator==(const Halfspace &other) const -> bool {
	return normal == other.normal && distance == other.distance;
}

auto Halfspace::decodeFrom(InputBitstream &stream) -> Halfspace {
  Halfspace plane;
  plane.normal.x() = decodeHalf(stream);
  plane.normal.y() = decodeHalf(stream);
  plane.normal.z() = decodeHalf(stream);
  plane.distance = decodeHalf(stream);
  return plane;
}

void Halfspace::encodeTo(OutputBitstream &stream) const {
  encodeHalf(normal.x(), stream);
  encodeHalf(normal.y(), stream);
  encodeHalf(normal.z(), stream);
  encodeHalf(distance, stream);
}

} // namespace TMIV::Metadata
