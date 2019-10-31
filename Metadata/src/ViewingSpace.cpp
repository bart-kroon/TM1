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
      primitiveShape.guardBandSize = stream.getFloat16();
    }
    if (orientationPresent) {
      primitiveShape.rotation = Common::Vec3f();
      primitiveShape.rotation.value().x() = stream.getFloat16();
      primitiveShape.rotation.value().y() = stream.getFloat16();
      primitiveShape.rotation.value().z() = stream.getFloat16();
    }
    if (directionConstraintPresent) {
      auto vdc = PrimitiveShape::ViewingDirectionConstraint();
      if (guardBandPresent) {
        vdc.guardBandDirectionSize = stream.getFloat16();
      }
      vdc.yawCenter = stream.getFloat16();
      vdc.yawRange = stream.getFloat16();
      vdc.pitchCenter = stream.getFloat16();
      vdc.pitchRange = stream.getFloat16();
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
      stream.putFloat16(Half(p.guardBandSize.value_or(0.F)));
    }
    if (orientationPresent) {
      const Common::Vec3f r = p.rotation.value_or(Common::Vec3f());
      stream.putFloat16(Half(r.x()));
      stream.putFloat16(Half(r.y()));
      stream.putFloat16(Half(r.z()));
    }
    if (directionConstraintPresent) {
      const auto vdc =
          p.viewingDirectionConstraint.value_or(PrimitiveShape::ViewingDirectionConstraint());
      if (guardBandPresent) {
        stream.putFloat16(Half(vdc.guardBandDirectionSize.value_or(0.F)));
      }
      stream.putFloat16(Half(vdc.yawCenter));
      stream.putFloat16(Half(vdc.yawRange));
      stream.putFloat16(Half(vdc.pitchCenter));
      stream.putFloat16(Half(vdc.pitchRange));
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
  cuboid.center.x() = stream.getFloat16();
  cuboid.center.y() = stream.getFloat16();
  cuboid.center.z() = stream.getFloat16();
  cuboid.size.x() = stream.getFloat16();
  cuboid.size.y() = stream.getFloat16();
  cuboid.size.z() = stream.getFloat16();
  return cuboid;
}

void Cuboid::encodeTo(OutputBitstream &stream) const {
  stream.putFloat16(Half(center.x()));
  stream.putFloat16(Half(center.y()));
  stream.putFloat16(Half(center.z()));
  stream.putFloat16(Half(size.x()));
  stream.putFloat16(Half(size.y()));
  stream.putFloat16(Half(size.z()));
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
  spheroid.center.x() = stream.getFloat16();
  spheroid.center.y() = stream.getFloat16();
  spheroid.center.z() = stream.getFloat16();
  spheroid.radius.x() = stream.getFloat16();
  spheroid.radius.y() = stream.getFloat16();
  spheroid.radius.z() = stream.getFloat16();
  return spheroid;
}

void Spheroid::encodeTo(OutputBitstream &stream) const {
  stream.putFloat16(Half(center.x()));
  stream.putFloat16(Half(center.y()));
  stream.putFloat16(Half(center.z()));
  stream.putFloat16(Half(radius.x()));
  stream.putFloat16(Half(radius.y()));
  stream.putFloat16(Half(radius.z()));
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
  plane.normal.x() = stream.getFloat16();
  plane.normal.y() = stream.getFloat16();
  plane.normal.z() = stream.getFloat16();
  plane.distance = stream.getFloat16();
  return plane;
}

void Halfspace::encodeTo(OutputBitstream &stream) const {
  stream.putFloat16(Half(normal.x()));
  stream.putFloat16(Half(normal.y()));
  stream.putFloat16(Half(normal.z()));
  stream.putFloat16(Half(distance));
}

} // namespace TMIV::Metadata
