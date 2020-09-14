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

#include <TMIV/ViewingSpace/ViewingSpaceEvaluator.h>

#include <TMIV/Common/Common.h>
#include <TMIV/ViewingSpace/SignedDistance.h>

#define _VERBOSE
#include <fstream>
#include <iostream>

using namespace TMIV::Common;

namespace TMIV::ViewingSpace {
using DirectionConstraint = MivBitstream::PrimitiveShape::ViewingDirectionConstraint;

struct ViewingSpaceEvaluation {
  SignedDistance sdBoundary;
  SignedDistance sdGuardBand;
  std::optional<DirectionConstraint> directionConstraint;
};

struct ViewingDirection {
  float yaw, pitch;
};

static auto viewingDirection(const QuatF &rotation) -> ViewingDirection {
  assert(normalized(rotation));
  static const Vec3f forwardAxis{1.F, 0.F, 0.F};
  const auto directionVector = rotate(forwardAxis, rotation);

  ViewingDirection d{};
  d.yaw = degperrad * std::atan2(directionVector.y(), directionVector.x());
  d.pitch = degperrad * std::acos(-directionVector.z());
  return d;
}

static auto yawDelta(const float b, const float a) -> float {
  float d = b - a;
  while (d > 180.F) {
    d -= 360.F;
  }
  while (d < -180.F) {
    d += 360.F;
  }
  return d;
}

static auto blend(const std::optional<DirectionConstraint> &ao,
                  const std::optional<DirectionConstraint> &bo, const float s)
    -> std::optional<DirectionConstraint> {
  if (!ao.has_value() && !bo.has_value()) {
    return ao;
  }
  const auto &a = ao.value_or(DirectionConstraint());
  const auto &b = bo.value_or(DirectionConstraint());
  assert(s >= 0.F && s <= 1.F);

  const float sa = 1.F - s;
  const float sb = s;

  DirectionConstraint result;
  if (!ao.has_value()) {
    result.directionRotation = b.directionRotation;
  } else if (!bo.has_value()) {
    result.directionRotation = a.directionRotation;
  } else {
    // choose the closer of the two quaternions leading to the same orientation so we avoid >180
    // degree rotations
    const auto d = dot(a.directionRotation, b.directionRotation);
    result.directionRotation =
        sa * a.directionRotation + (d >= 0.F ? sb : -sb) * b.directionRotation;
  }
  result.yawRange = sa * a.yawRange + sb * b.yawRange;
  result.pitchRange = sa * a.pitchRange + sb * b.pitchRange;
  result.guardBandDirectionSize =
      sa * a.guardBandDirectionSize.value_or(0.F) + sb * b.guardBandDirectionSize.value_or(0.F);
  return result;
}

static auto evaluate(const MivBitstream::PrimitiveShape &shape, const ViewingParams &viewingParams)
    -> ViewingSpaceEvaluation {
  ViewingSpaceEvaluation result;
  result.sdBoundary = signedDistance(shape, viewingParams.viewPosition);
  result.sdGuardBand = SignedDistance(result.sdBoundary.value + shape.guardBandSize.value_or(0.F));
  result.directionConstraint = shape.viewingDirectionConstraint;
  return result;
}

static auto evaluateAddition(const MivBitstream::PrimitiveShapeVector &primitives,
                             const ViewingParams &viewingParams) -> ViewingSpaceEvaluation {
  ViewingSpaceEvaluation result;
  float accumulatedDirectionWeight = 0.F;
  for (const auto &primitive : primitives) {
    const auto e = evaluate(primitive, viewingParams);
    result.sdBoundary = result.sdBoundary + e.sdBoundary;
    result.sdGuardBand = result.sdGuardBand + e.sdGuardBand;
    if (e.sdBoundary.isInside()) {
      const float weight = -e.sdBoundary.value;
      accumulatedDirectionWeight += weight;
      result.directionConstraint = blend(result.directionConstraint, e.directionConstraint,
                                         weight / accumulatedDirectionWeight);
    }
  }
  return result;
}

namespace MiscInterpolation {
auto normalToPlane(Vec3f pos1, Vec3f pos2, Vec3f pos3) -> Vec3f {
  Vec3f normal = cross(pos2 - pos1, pos3 - pos1);
  normal /= norm(normal);
  return normal;
}

auto orthogonalPlane(Vec3f pos1, Vec3f pos2, Vec4f predecessor) -> Vec4f {
  Vec3f normal = (pos1 - pos2) / norm(pos2 - pos1);
  Vec3f normal_pred({predecessor[0], predecessor[1], predecessor[2]});
  if (dot(normal, normal_pred) < 0) {
    normal *= -1;
  }
  return Vec4f({normal[0], normal[1], normal[2], -dot(pos1, normal)});
}

auto bisectingPlane(Vec3f pos1, Vec3f pos2, Vec3f pos3, Vec4f predecessor) -> Vec4f {
  Vec3f vec1 = pos1 - pos2;
  vec1 /= norm(vec1);
  Vec3f vec3 = pos3 - pos2;
  vec3 /= norm(vec3);
  Vec3f bisecting_dir = vec1 + vec3;
  if (norm(bisecting_dir) > 0) {
    bisecting_dir /= norm(bisecting_dir);
    Vec3f normal1 = normalToPlane(pos1, pos2, pos3);
    Vec3f normal2 = cross(bisecting_dir, normal1);
    Vec3f normal_pred({predecessor[0], predecessor[1], predecessor[2]});
    if (dot(normal2, normal_pred) < 0) {
      normal2 *= -1;
    }
    return Vec4f({normal2[0], normal2[1], normal2[2], -dot(pos2, normal2)});
  }
  { // note: degenerate case, 3 aligned points
    return orthogonalPlane(pos2, pos1, predecessor);
  }
}

auto distanceToPlane(Vec4f plane, Vec3f point) -> float {
  Vec3f normal({plane[0], plane[1], plane[2]});
  return dot(point, normal) + plane[3];
}

auto projectedPointOnPlane(Vec3f a, Vec3f b, Vec3f normal, Vec3f point) -> Vec3f {
  Vec3f pp;
  if (a == b) {
    pp = a;
  } else {
    Vec4f pplane = Vec4f({normal[0], normal[1], normal[2], -dot(point, normal)});
    float aap = std::abs(distanceToPlane(pplane, a));
    float bbp = std::abs(distanceToPlane(pplane, b));
    float app = aap / (aap + bbp);
    pp = a + app * (b - a);
  }
  return pp;
}

static auto computeBisectPlanes(const MivBitstream::PrimitiveShapeVector &primitives)
    -> std::vector<Vec4f> {
  auto nvb = primitives.size();
  std::vector<Vec3f> center(nvb);
  for (size_t i = 0; i < nvb; i++) {
    if (primitives[i].shapeType() == MivBitstream::PrimitiveShapeType::spheroid) {
      center[i] = std::get<MivBitstream::Spheroid>(primitives[i].primitive).center;
    } else if (primitives[i].shapeType() == MivBitstream::PrimitiveShapeType::cuboid) {
      center[i] = std::get<MivBitstream::Cuboid>(primitives[i].primitive).center;
    }
  }

  std::vector<Vec4f> bisect;
  Vec4f predecessor;
  for (size_t i = 0; i < nvb; i++) {
    if (i == 0) {
      predecessor = {1, 0, 0, 0};
      bisect.push_back(MiscInterpolation::orthogonalPlane(center[0], center[1], predecessor));
    } else if (i + 1 < nvb) {
      predecessor = bisect[i - 1];
      bisect.push_back(
          MiscInterpolation::bisectingPlane(center[i - 1], center[i], center[i + 1], predecessor));
    } else {
      predecessor = bisect[i - 1];
      bisect.push_back(MiscInterpolation::orthogonalPlane(center[i], center[i - 1], predecessor));
    }
  }
  return bisect;
}
} // namespace MiscInterpolation

auto interpolateShape(const MivBitstream::PrimitiveShape a, const MivBitstream::PrimitiveShape b,
                      Vec3f center, float w) -> MivBitstream::PrimitiveShape {
  MivBitstream::PrimitiveShape output(a);
  assert(a.shapeType() == b.shapeType());
  assert(a.shapeType() == MivBitstream::PrimitiveShapeType::spheroid ||
         (a.shapeType() == MivBitstream::PrimitiveShapeType::cuboid));

  // dimension and position
  if (a.shapeType() == MivBitstream::PrimitiveShapeType::spheroid) {
    Vec3f ra = std::get<MivBitstream::Spheroid>(a.primitive).radius;
    Vec3f rb = std::get<MivBitstream::Spheroid>(b.primitive).radius;
    std::get<MivBitstream::Spheroid>(output.primitive).radius = (1.F - w) * ra + w * rb;
    std::get<MivBitstream::Spheroid>(output.primitive).center = center;
#ifdef _VERBOSE
    std::cout << "interpolated shape:" << std::endl;
    std::cout << "  center = " << center << std::endl;
    std::cout << "  radius = " << std::get<MivBitstream::Spheroid>(output.primitive).radius
              << std::endl;
#endif
  } else if (a.shapeType() == MivBitstream::PrimitiveShapeType::cuboid) {
    Vec3f sa = std::get<MivBitstream::Cuboid>(a.primitive).size;
    Vec3f sb = std::get<MivBitstream::Cuboid>(b.primitive).size;
    std::get<MivBitstream::Cuboid>(output.primitive).size = (1. - w) * sa + w * sb;
    std::get<MivBitstream::Spheroid>(output.primitive).center = center;
  }

  // rotation
  const auto rota = a.rotation.value_or(QuatF{0.F, 0.F, 0.F, 1.F});
  const auto rotb = b.rotation.value_or(QuatF{0.F, 0.F, 0.F, 1.F});
  output.rotation = (1. - w) * rota + w * rotb;
#ifdef _VERBOSE
  std::cout << "  rotation = " << output.rotation.value() << std::endl;
#endif
  // guard band size
  const float gba = a.guardBandSize.value_or(0.F);
  const float gbb = b.guardBandSize.value_or(0.F);
  output.guardBandSize = (1.F - w) * gba + w * gbb;
#ifdef _VERBOSE
  std::cout << "  guard band = " << output.guardBandSize.value() << std::endl;
#endif
  // viewing direction constraint
  output.viewingDirectionConstraint =
      blend(a.viewingDirectionConstraint.value(), b.viewingDirectionConstraint.value(), w);
#ifdef _VERBOSE
  std::cout << "  direction rotation = "
            << output.viewingDirectionConstraint.value().directionRotation << std::endl
            << "  yaw range = " << output.viewingDirectionConstraint.value().yawRange << std::endl
            << "  pitch range = " << output.viewingDirectionConstraint.value().pitchRange
            << std::endl;
#endif
  // directional guard band size
  const float vgba = a.viewingDirectionConstraint.value().guardBandDirectionSize.value_or(0.F);
  const float vgbb = b.viewingDirectionConstraint.value().guardBandDirectionSize.value_or(0.F);
  output.viewingDirectionConstraint.value().guardBandDirectionSize = (1.F - w) * vgba + w * vgbb;
#ifdef _VERBOSE
  std::cout << "  viewing direction guard band = "
            << output.viewingDirectionConstraint.value().guardBandDirectionSize.value()
            << std::endl;
#endif
  return output;
}
static auto evaluateInterpolation(const MivBitstream::PrimitiveShapeVector &primitives,
                                  const ViewingParams &viewingParams) -> ViewingSpaceEvaluation {
  ViewingSpaceEvaluation result;

  // computation of bisecting planes (once per sequence)
  static auto bisect = MiscInterpolation::computeBisectPlanes(primitives);

  // interpolate primitive shape and evaluate distance
  int nvb = static_cast<int>(primitives.size());
  if (nvb > 1) {
    Vec2i segment;
    Vec3f pos = viewingParams.viewPosition;

    // distance to shape centers
    std::vector<Vec3f> center(nvb);
    std::vector<float> dist;
    for (auto i = 0; i < nvb; i++) {
      if (primitives[i].shapeType() == MivBitstream::PrimitiveShapeType::spheroid) {
        center[i] = std::get<MivBitstream::Spheroid>(primitives[i].primitive).center;
      } else if (primitives[i].shapeType() == MivBitstream::PrimitiveShapeType::cuboid) {
        center[i] = std::get<MivBitstream::Cuboid>(primitives[i].primitive).center;
      }
      dist.push_back(norm(pos - center[i]));
    }

    // find closest shape
    std::vector<float>::iterator it;
    it = std::min_element(dist.begin(), dist.end());
    int closest = static_cast<int>(std::distance(dist.begin(), it));

    // find segment of attachment
    if (closest == 0) {
      segment = (std::signbit(MiscInterpolation::distanceToPlane(bisect[0], pos)) ==
                 std::signbit(MiscInterpolation::distanceToPlane(bisect[1], pos)))
                    ? Vec2i({0, 0})
                    : Vec2i({0, 1});
    } else if (closest == nvb - 1) {
      segment = (std::signbit(MiscInterpolation::distanceToPlane(bisect[nvb - 1], pos)) ==
                 std::signbit(MiscInterpolation::distanceToPlane(bisect[nvb - 2], pos)))
                    ? Vec2i({nvb - 1, nvb - 1})
                    : Vec2i({nvb - 2, nvb - 1});
    } else {
      segment = (std::signbit(MiscInterpolation::distanceToPlane(bisect[closest - 1], pos)) ==
                 std::signbit(MiscInterpolation::distanceToPlane(bisect[closest], pos)))
                    ? Vec2i({closest, closest + 1})
                    : Vec2i({closest - 1, closest});
    }

    // compute position of interpolated shape within segment of attachment
    int start(segment[0]);
    int end(segment[1]);
    float w = (start != end) ? dist[start] / (dist[start] + dist[end]) : 0;
    Vec3f n_start({bisect[start][0], bisect[start][1], bisect[start][2]});
    Vec3f n_end({bisect[end][0], bisect[end][1], bisect[end][2]});
    Vec3f n_proj = w * n_end + (1 - w) * n_start;
#ifdef _VERBOSE
    std::cout << "segment <" << start << "," << end << "> ";
    std::cout << "weight = " << w << std::endl;
#endif
    Vec3f p = MiscInterpolation::projectedPointOnPlane(center[start], center[end], n_proj, pos);

    // interpolate shape dimension, rotation, guard band size, viewing direction constraint
    MivBitstream::PrimitiveShape shape = interpolateShape(primitives[start], primitives[end], p, w);

    // evaluate distance to interpolated shape
    result = evaluate(shape, viewingParams);

  } else {
    result = evaluate(primitives[0], viewingParams);
  }
#ifdef _VERBOSE
  std::cout << "signed distance = " << result.sdBoundary.value << std::endl;
  std::cout << "signed distance + guard band = " << result.sdGuardBand.value << std::endl;
  std::cout << "isInside = " << result.sdBoundary.isInside() << std::endl;
#endif
  return result;
}

static auto evaluate(const MivBitstream::ElementaryShape &shape, const ViewingParams &viewingParams)
    -> ViewingSpaceEvaluation {
  if (shape.primitiveOperation == MivBitstream::PrimitiveShapeOperation::add) {
    return evaluateAddition(shape.primitives, viewingParams);
  }
  if (shape.primitiveOperation == MivBitstream::PrimitiveShapeOperation::interpolate) {
    return evaluateInterpolation(shape.primitives, viewingParams);
  }
  abort();
}

static auto distanceInclusion(const SignedDistance sdBoundary, const SignedDistance sdGuard)
    -> float {
  if (sdGuard.isInside()) {
    return 1.F;
  }
  if (sdBoundary.isOutside()) {
    return 0.F;
  }
  const float guardBandDepth = sdGuard.value - sdBoundary.value; // note sdGuard > sdBoundary
  const float inclusion = -sdBoundary.value / guardBandDepth;
  assert(inRange(inclusion, 0.F, 1.F));
  return inclusion;
}

static auto angleInclusion(const float deltaAngle, const float range, const float guardBand)
    -> float {
  const float absDelta = std::abs(deltaAngle);
  const float maxDelta = 0.5F * range;
  const float guardStart = maxDelta - guardBand;
  if (absDelta <= guardStart) {
    return 1.F;
  }
  if (absDelta > maxDelta) {
    return 0.F;
  }
  const float inclusion = (maxDelta - absDelta) / guardBand;

  assert(inRange(inclusion, 0.F, 1.F));
  return inclusion;
}

auto ViewingSpaceEvaluator::computeInclusion(const MivBitstream::ViewingSpace &viewingSpace,
                                             const ViewingParams &viewingParams) -> float {
  ViewingSpaceEvaluation global;
  float accumulatedDirectionWeight = 0.F;
  for (const auto &e : viewingSpace.elementaryShapes) {
    const auto eval = evaluate(e.second, viewingParams);
    if (e.first == MivBitstream::ElementaryShapeOperation::add) {
      global.sdBoundary += eval.sdBoundary;
      global.sdGuardBand += eval.sdGuardBand;
    }
    if (e.first == MivBitstream::ElementaryShapeOperation::subtract) {
      global.sdBoundary -= eval.sdGuardBand;
      global.sdGuardBand -= eval.sdBoundary;
    }
    if (e.first == MivBitstream::ElementaryShapeOperation::intersect) {
      global.sdBoundary &= eval.sdBoundary;
      global.sdGuardBand &= eval.sdGuardBand;
    }
    if (eval.sdBoundary.isInside()) {
      const float weight = -eval.sdBoundary.value;
      accumulatedDirectionWeight += weight;
      global.directionConstraint = blend(global.directionConstraint, eval.directionConstraint,
                                         weight / accumulatedDirectionWeight);
    }
  }
  if (global.directionConstraint.has_value()) {
    global.directionConstraint.value().directionRotation =
        normalize(global.directionConstraint.value().directionRotation);
  }

  const auto &dc = global.directionConstraint.value_or(DirectionConstraint());

  const auto dcd = viewingDirection(dc.directionRotation);
  const auto vpd = viewingDirection(viewingParams.viewRotation);

  const float kPosition = distanceInclusion(global.sdBoundary, global.sdGuardBand);
  const float kYaw = angleInclusion(yawDelta(dcd.yaw, vpd.yaw), dc.yawRange,
                                    dc.guardBandDirectionSize.value_or(0.F));
  const float kPitch =
      angleInclusion(vpd.pitch - dcd.pitch, dc.pitchRange, dc.guardBandDirectionSize.value_or(0.F));
  const float result = kPosition * kYaw * kPitch;

#ifdef _VERBOSE
  std::cout << "kPosition = " << kPosition << std::endl;
  if (kPosition != 0.F) {
    std::cout << "kYaw = " << kYaw << std::endl;
    std::cout << "kPitch = " << kPitch << std::endl;
  }
#endif

  assert(inRange(result, 0.F, 1.F));
  return result;
}

} // namespace TMIV::ViewingSpace
