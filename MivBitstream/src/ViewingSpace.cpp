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

#include <TMIV/MivBitstream/ViewingSpace.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Common.h>
#include <TMIV/Common/Half.h>
#include <TMIV/MivBitstream/verify.h>

namespace TMIV::MivBitstream {
using Common::Half;
using Common::Json;
using Common::QuatF;

static auto decodeRotation(Common::InputBitstream &stream) -> Common::QuatF {
  Common::QuatF q;
  q.x() = stream.getFloat16();
  q.y() = stream.getFloat16();
  q.z() = stream.getFloat16();
  q.w() = 0.F;
  q.w() = std::sqrt(1.F - norm2(q));
  return q;
}

static void encodeRotation(const Common::QuatF &q, Common::OutputBitstream &stream) {
  assert(normalized(q, 1.0E-3F));
  stream.putFloat16(Common::Half(q.x()));
  stream.putFloat16(Common::Half(q.y()));
  stream.putFloat16(Common::Half(q.z()));
}

static auto equalRotation(const Common::QuatF &a, const Common::QuatF &b) -> bool {
  assert(normalized(a, 1.0E-3F) && normalized(b, 1.0E-3F));
  const float d = dot(a, b);
  return d > 0.9999F;
}

auto operator<<(std::ostream &stream, const ViewingSpace &viewingSpace) -> std::ostream & {
  stream << "Viewing space:" << std::endl;
  for (const auto &s : viewingSpace.elementaryShapes) {
    stream << (s.first == ElementaryShapeOperation::add ? "add " : "subtract ");
    stream << '(' << s.second << ')' << std::endl;
  }
  return stream;
}

auto ViewingSpace::operator==(const ViewingSpace &other) const -> bool {
  return (elementaryShapes == other.elementaryShapes);
}

auto ViewingSpace::decodeFrom(Common::InputBitstream &stream,
                              const TMIV::MivBitstream::ViewParamsList &viewParamsList)
    -> ViewingSpace {
  ViewingSpace vs;
  auto numShapes = stream.getUExpGolomb<size_t>() + 1;
  vs.elementaryShapes.reserve(numShapes);
  for (size_t i = 0; i < numShapes; ++i) {
    const auto op = stream.readBits<ElementaryShapeOperation>(2);
    const auto shape = ElementaryShape::decodeFrom(stream, viewParamsList);
    vs.elementaryShapes.emplace_back(op, shape);
  }
  return vs;
}

void ViewingSpace::encodeTo(Common::OutputBitstream &stream,
                            const TMIV::MivBitstream::ViewParamsList & /*viewParamsList*/) const {
  VERIFY_MIVBITSTREAM(!elementaryShapes.empty());
  stream.putUExpGolomb(elementaryShapes.size() - 1);
  for (const auto &shape : elementaryShapes) {
    stream.writeBits(shape.first, 2);
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

auto ElementaryShape::decodeFrom(Common::InputBitstream &stream,
                                 const TMIV::MivBitstream::ViewParamsList &viewParamsList)
    -> ElementaryShape {
  ElementaryShape elementaryShape;
  const auto numPrimitives = stream.readBits<size_t>(8) + 1;
  elementaryShape.primitiveOperation = stream.readBits<PrimitiveShapeOperation>(1);
  const auto guardBandPresent = stream.getFlag();
  const auto orientationPresent = stream.getFlag();
  const auto directionConstraintPresent = stream.getFlag();
  const auto cameraInferred = stream.getFlag();
  elementaryShape.primitives.reserve(numPrimitives);
  for (size_t i = 0; i < numPrimitives; ++i) {
    TMIV::Common::Vec3f c{};
    if (cameraInferred) {
      int view = static_cast<int>(stream.getUint16());
      c = viewParamsList[view].ce.position();
    }
    PrimitiveShape primitiveShape;
    const auto shapeType = stream.readBits<PrimitiveShapeType>(2);
    switch (shapeType) {
    case PrimitiveShapeType::cuboid:
      primitiveShape.primitive = Cuboid::decodeFrom(stream, cameraInferred, c);
      break;
    case PrimitiveShapeType::spheroid:
      primitiveShape.primitive = Spheroid::decodeFrom(stream, cameraInferred, c);
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
      if (!cameraInferred) {
        primitiveShape.rotation = decodeRotation(stream);
      }
    }
    if (directionConstraintPresent) {
      auto vdc = PrimitiveShape::ViewingDirectionConstraint();
      if (guardBandPresent) {
        vdc.guardBandDirectionSize = stream.getFloat16();
      }
      if (!cameraInferred) {
        vdc.directionRotation = decodeRotation(stream);
      }
      vdc.yawRange = stream.getFloat16();
      vdc.pitchRange = stream.getFloat16();
      primitiveShape.viewingDirectionConstraint = vdc;
    }
    elementaryShape.primitives.emplace_back(primitiveShape);
  }
  return elementaryShape;
}

void ElementaryShape::encodeTo(Common::OutputBitstream &stream) const {
  VERIFY_MIVBITSTREAM(!primitives.empty());
  bool guardBandPresent{};
  bool orientationPresent{};
  bool directionConstraintPresent{};
  bool cameraInferred = !inferringViews.empty();
  for (const auto &p : primitives) {
    guardBandPresent |= p.guardBandSize.has_value();
    orientationPresent |= p.rotation.has_value();
    if (p.viewingDirectionConstraint.has_value()) {
      directionConstraintPresent |= true;
      guardBandPresent |= p.viewingDirectionConstraint.value().guardBandDirectionSize.has_value();
    }
  }
  stream.writeBits(primitives.size() - 1, 8);
  stream.writeBits(primitiveOperation, 1);
  stream.putFlag(guardBandPresent);
  stream.putFlag(orientationPresent);
  stream.putFlag(directionConstraintPresent);
  stream.putFlag(cameraInferred);
  for (size_t idx = 0; idx < primitives.size(); idx++) {
    const auto &p = primitives[idx];
    if (cameraInferred) {
      stream.putUint16(static_cast<uint16_t>(inferringViews[idx]));
    }
    stream.writeBits(p.shapeType(), 2);
    visit([&](const auto &x) { x.encodeTo(stream, cameraInferred); }, p.primitive);
    if (guardBandPresent) {
      stream.putFloat16(Common::Half(p.guardBandSize.value_or(0.F)));
    }
    if (orientationPresent) {
      if (!cameraInferred) {
        encodeRotation(p.rotation.value(), stream);
      }
    }
    if (directionConstraintPresent) {
      const auto vdc =
          p.viewingDirectionConstraint.value_or(PrimitiveShape::ViewingDirectionConstraint());
      if (guardBandPresent) {
        stream.putFloat16(Common::Half(vdc.guardBandDirectionSize.value_or(0.F)));
      }
      if (!cameraInferred) {
        encodeRotation(vdc.directionRotation, stream);
      }
      stream.putFloat16(Common::Half(vdc.yawRange));
      stream.putFloat16(Common::Half(vdc.pitchRange));
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
    stream << " direction rotation " << vdc.directionRotation << " yaw range " << vdc.yawRange
           << " pitch range " << vdc.pitchRange << "+/-" << 0.5F * vdc.pitchRange;
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
  if (!equalRotation(rotation.value_or(Common::QuatF{0.F, 0.F, 0.F, 1.F}),
                     other.rotation.value_or(Common::QuatF{0.F, 0.F, 0.F, 1.F}))) {
    return false;
  }
  if (viewingDirectionConstraint != other.viewingDirectionConstraint) {
    return false;
  }
  return true;
}

auto PrimitiveShape::ViewingDirectionConstraint::operator==(
    const ViewingDirectionConstraint &other) const -> bool {
  if (guardBandDirectionSize != other.guardBandDirectionSize) {
    return false;
  }
  if (!equalRotation(directionRotation, other.directionRotation)) {
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

auto Cuboid::decodeFrom(Common::InputBitstream &stream, bool cameraInferred, TMIV::Common::Vec3f c)
    -> Cuboid {
  Cuboid cuboid;
  if (cameraInferred) {
    cuboid.center = c;
  } else {
    cuboid.center.x() = stream.getFloat16();
    cuboid.center.y() = stream.getFloat16();
    cuboid.center.z() = stream.getFloat16();
  }
  cuboid.size.x() = stream.getFloat16();
  cuboid.size.y() = stream.getFloat16();
  cuboid.size.z() = stream.getFloat16();
  return cuboid;
}

void Cuboid::encodeTo(Common::OutputBitstream &stream, bool cameraInferred) const {
  if (!cameraInferred) {
    stream.putFloat16(Common::Half(center.x()));
    stream.putFloat16(Common::Half(center.y()));
    stream.putFloat16(Common::Half(center.z()));
  }
  stream.putFloat16(Common::Half(size.x()));
  stream.putFloat16(Common::Half(size.y()));
  stream.putFloat16(Common::Half(size.z()));
}

auto operator<<(std::ostream &stream, const Spheroid &spheroid) -> std::ostream & {
  stream << "spheroid " << spheroid.center << " radius " << spheroid.radius;
  return stream;
}

auto Spheroid::operator==(const Spheroid &other) const -> bool {
  return center == other.center && radius == other.radius;
}

auto Spheroid::decodeFrom(Common::InputBitstream &stream, bool cameraInferred,
                          TMIV::Common::Vec3f c) -> Spheroid {
  Spheroid spheroid;
  if (cameraInferred) {
    spheroid.center = c;
  } else {
    spheroid.center.x() = stream.getFloat16();
    spheroid.center.y() = stream.getFloat16();
    spheroid.center.z() = stream.getFloat16();
  }
  spheroid.radius.x() = stream.getFloat16();
  spheroid.radius.y() = stream.getFloat16();
  spheroid.radius.z() = stream.getFloat16();
  return spheroid;
}

void Spheroid::encodeTo(Common::OutputBitstream &stream, bool cameraInferred) const {
  if (!cameraInferred) {
    stream.putFloat16(Common::Half(center.x()));
    stream.putFloat16(Common::Half(center.y()));
    stream.putFloat16(Common::Half(center.z()));
  }
  stream.putFloat16(Common::Half(radius.x()));
  stream.putFloat16(Common::Half(radius.y()));
  stream.putFloat16(Common::Half(radius.z()));
}

auto operator<<(std::ostream &stream, const Halfspace &halfspace) -> std::ostream & {
  stream << "halfspace " << halfspace.normal << " distance " << halfspace.distance;
  return stream;
}

auto Halfspace::operator==(const Halfspace &other) const -> bool {
  return normal == other.normal && distance == other.distance;
}

auto Halfspace::decodeFrom(Common::InputBitstream &stream) -> Halfspace {
  Halfspace plane;
  plane.normal.x() = stream.getFloat16();
  plane.normal.y() = stream.getFloat16();
  plane.normal.z() = stream.getFloat16();
  plane.distance = stream.getFloat16();
  return plane;
}

void Halfspace::encodeTo(Common::OutputBitstream &stream, bool /*cameraInferred*/) const {
  stream.putFloat16(Common::Half(normal.x()));
  stream.putFloat16(Common::Half(normal.y()));
  stream.putFloat16(Common::Half(normal.z()));
  stream.putFloat16(Common::Half(distance));
}

auto ViewingSpace::loadFromJson(const Common::Json &node, const Common::Json &config)
    -> ViewingSpace {
  auto parseOperation = [](const std::string &str) -> auto {
    if (str == "add") {
      return ElementaryShapeOperation::add;
    }
    if (str == "subtract") {
      return ElementaryShapeOperation::subtract;
    };
    if (str == "intersect") {
      return ElementaryShapeOperation::intersect;
    }
    throw std::runtime_error("Invalid elementary shape operation in the metadata JSON file");
  };

  ViewingSpace viewingSpace{};
  const auto &elementaryShapes = node.require("ElementaryShapes").as<Json::Array>();

  for (const auto & elementaryShape : elementaryShapes) {
    viewingSpace.elementaryShapes.emplace_back(
        parseOperation(elementaryShape.require("ElementaryShapeOperation").as<std::string>()),
        ElementaryShape::loadFromJson(elementaryShape.require("ElementaryShape"), config));
  }

  // consolidate the optional values across all primitives in each elementary shape
  for (auto &elementaryShape : viewingSpace.elementaryShapes) {
    bool guardBandPresent{};
    bool rotationPresent{};
    bool directionConstraintPresent{};
    for (auto &primitive : elementaryShape.second.primitives) {
      guardBandPresent |= primitive.guardBandSize.has_value();
      rotationPresent |= primitive.rotation.has_value();
      if (primitive.viewingDirectionConstraint.has_value()) {
        directionConstraintPresent |= true;
        guardBandPresent |=
            primitive.viewingDirectionConstraint.value().guardBandDirectionSize.has_value();
      }
    }
    for (auto &primitive : elementaryShape.second.primitives) {
      if (guardBandPresent && !primitive.guardBandSize.has_value()) {
        primitive.guardBandSize = 0.F;
      }
      if (rotationPresent && !primitive.rotation.has_value()) {
        primitive.rotation = Common::QuatF();
      }
      if (directionConstraintPresent) {
        if (!primitive.viewingDirectionConstraint.has_value()) {
          primitive.viewingDirectionConstraint = PrimitiveShape::ViewingDirectionConstraint();
        }
        if (guardBandPresent &&
            !primitive.viewingDirectionConstraint.value().guardBandDirectionSize.has_value()) {
          primitive.viewingDirectionConstraint.value().guardBandDirectionSize = 0.F;
        }
      }
    }
  }
  return viewingSpace;
}

auto ElementaryShape::loadFromJson(const Common::Json &node, const Common::Json &config)
    -> ElementaryShape {
  auto parseOperation = [](const std::string &str) -> auto {
    if (str == "add") {
      return PrimitiveShapeOperation::add;
    }
    if (str == "interpolate") {
      return PrimitiveShapeOperation::interpolate;
    };
    throw std::runtime_error("Invalid primitive shape operation in the metadata JSON file");
  };

  ElementaryShape elementaryShape{};

  // added for m52412 inferred_views implementation
  bool inferredView = false;
  const auto sourceCameraNames = (config.require("SourceCameraNames").asVector<std::string>());
  if (const auto &subsubnode = node.optional("InferringViews")) {
    std::vector<std::string> views = subsubnode.asVector<std::string>();
    for (const auto &v : views) {
      size_t idx = 0;
      for (; idx < sourceCameraNames.size(); idx++) {
        if (v == sourceCameraNames[idx]) {
          break;
        }
      }
      if (idx == sourceCameraNames.size()) {
        throw std::runtime_error("Invalid inferred view in the metadata JSON file");
      }
      elementaryShape.inferringViews.push_back(static_cast<int>(idx));
    }
    inferredView = true;
  }

  // primitive shape operation
  elementaryShape.primitiveOperation =
      parseOperation(node.require("PrimitiveShapeOperation").as<std::string>());

  // primitive shapes
  const auto &primitiveShapes = node.require("PrimitiveShapes").as<Json::Array>();
  for (const auto & primitiveShape : primitiveShapes) {
    elementaryShape.primitives.push_back(
        PrimitiveShape::loadFromJson(primitiveShape, inferredView));
  }

  // check consistency
  if (inferredView &&
      (elementaryShape.primitives.size() != elementaryShape.inferringViews.size())) {
    throw std::runtime_error(
        "Incompatible number of inferring views and primitive shapes in the metadata JSON file");
  }

  return elementaryShape;
}

auto PrimitiveShape::loadFromJson(const Common::Json &node, bool inferredView) -> PrimitiveShape {
  PrimitiveShape primitiveShape{};
  const auto &shapeType = node.require("PrimitiveShapeType").as<std::string>();
  if (shapeType == "cuboid") {
    primitiveShape.primitive = Cuboid::loadFromJson(node, inferredView);
  }
  if (shapeType == "spheroid") {
    primitiveShape.primitive = Spheroid::loadFromJson(node, inferredView);
  }
  if (shapeType == "halfspace") {
    primitiveShape.primitive = Halfspace::loadFromJson(node, inferredView);
  }
  if (const auto &subnode = node.optional("GuardBandSize")) {
    primitiveShape.guardBandSize = subnode.as<float>();
  }
  if (const auto &subnode = node.optional("Rotation")) {
    primitiveShape.rotation = Common::euler2quat(Common::radperdeg * subnode.asVec<float, 3>());
  }
  if (const auto &subnode = node.optional("ViewingDirectionConstraint")) {
    primitiveShape.viewingDirectionConstraint = PrimitiveShape::ViewingDirectionConstraint();
    if (const auto &subsubnode = subnode.optional("GuardBandDirectionSize")) {
      primitiveShape.viewingDirectionConstraint.value().guardBandDirectionSize =
          subsubnode.as<float>();
    }
    if (!inferredView) {
      const float directionYaw = subnode.require("YawCenter").as<float>();
      const float directionPitch = subnode.require("PitchCenter").as<float>();
      primitiveShape.viewingDirectionConstraint.value().directionRotation =
          Common::euler2quat(Common::radperdeg * Common::Vec3f{directionYaw, directionPitch, 0.F});
    }
    primitiveShape.viewingDirectionConstraint.value().yawRange =
        subnode.require("YawRange").as<float>();
    primitiveShape.viewingDirectionConstraint.value().pitchRange =
        subnode.require("PitchRange").as<float>();
  }
  return primitiveShape;
}

auto Cuboid::loadFromJson(const Common::Json &node, bool inferredView) -> Cuboid {
  Cuboid cuboid;
  if (!inferredView) {
    cuboid.center = node.require("Center").asVec<float, 3>();
  }
  cuboid.size = node.require("Size").asVec<float, 3>();
  return cuboid;
}

auto Spheroid::loadFromJson(const Common::Json &node, bool inferredView) -> Spheroid {
  Spheroid spheroid;
  if (!inferredView) {
    spheroid.center = node.require("Center").asVec<float, 3>();
  }
  spheroid.radius = node.require("Radius").asVec<float, 3>();
  return spheroid;
}

auto Halfspace::loadFromJson(const Common::Json &node, bool /*inferredView*/) -> Halfspace {
  Halfspace plane;
  plane.normal = node.require("Normal").asVec<float, 3>();
  plane.distance = node.require("Distance").as<float>();
  return plane;
}

} // namespace TMIV::MivBitstream
