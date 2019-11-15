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

#include <TMIV/ViewingSpace/ViewingSpaceEvaluator.h>

#include <TMIV/ViewingSpace/SignedDistance.h>

using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::ViewingSpace {

using DirectionConstraint = PrimitiveShape::ViewingDirectionConstraint;

struct ViewingSpaceEvaluation {
  SignedDistance sdBoundary;
  SignedDistance sdGuardBand;
  DirectionConstraint directionConstraint;
};

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

static auto blend(const DirectionConstraint &a, const DirectionConstraint &b, const float s) {
  assert(a.pitchCenter >= -90.F && a.pitchCenter <= 90.F);
  assert(b.pitchCenter >= -90.F && b.pitchCenter <= 90.F);
  assert(s >= 0.F && s <= 1.F);

  const float sa = 1.F - s;
  const float sb = s;

  DirectionConstraint result;
  result.yawCenter = a.yawCenter + s * yawDelta(b.yawCenter, a.yawCenter);
  result.yawRange = sa * a.yawRange + sb * b.yawRange;
  result.pitchCenter = sa * a.pitchCenter + sb * b.pitchCenter;
  result.pitchRange = sa * a.pitchRange + sb * b.pitchRange;
  result.guardBandDirectionSize =
      sa * a.guardBandDirectionSize.value_or(0.F) + sb * b.guardBandDirectionSize.value_or(0.F);
  return result;
}

static auto evaluate(const PrimitiveShape &shape, const ViewingParams &viewingParams)
    -> ViewingSpaceEvaluation {
  ViewingSpaceEvaluation result;
  result.sdBoundary = signedDistance(shape, viewingParams.viewPosition);
  result.sdGuardBand = SignedDistance(result.sdBoundary.value + shape.guardBandSize.value_or(0.F));
  result.directionConstraint = shape.viewingDirectionConstraint.value_or(DirectionConstraint());
  return result;
}

static auto evaluateAddition(const PrimitiveShapeVector &primitives,
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

static auto evaluateInterpolation(const PrimitiveShapeVector &primitives,
                                  const ViewingParams &viewingParams) -> ViewingSpaceEvaluation {
  ViewingSpaceEvaluation result;
  // TODO implement interpolation
  return result;
}

static auto evaluate(const ElementaryShape &shape, const ViewingParams &viewingParams)
    -> ViewingSpaceEvaluation {
  if (shape.primitiveOperation == PrimitiveShapeOperation::add) {
    return evaluateAddition(shape.primitives, viewingParams);
  }
  if (shape.primitiveOperation == PrimitiveShapeOperation::interpolate) {
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
  const float inclusion = (absDelta - guardStart) / guardBand;
  assert(inRange(inclusion, 0.F, 1.F));
  return inclusion;
}

auto ViewingSpaceEvaluator::computeInclusion(const Metadata::ViewingSpace &viewingSpace,
                                             const ViewingParams &viewingParams) -> float {
  ViewingSpaceEvaluation global;
  float accumulatedDirectionWeight = 0.F;
  for (const auto &e : viewingSpace.elementaryShapes) {
    const auto eval = evaluate(e.second, viewingParams);
    if (e.first == ElementaryShapeOperation::add) {
      global.sdBoundary += eval.sdBoundary;
      global.sdGuardBand += eval.sdGuardBand;
    }
    if (e.first == ElementaryShapeOperation::subtract) {
      global.sdBoundary -= eval.sdBoundary;
      global.sdGuardBand -= eval.sdGuardBand;
    }
    if (eval.sdBoundary.isInside()) {
      const float weight = -eval.sdBoundary.value;
      accumulatedDirectionWeight += weight;
      global.directionConstraint = blend(global.directionConstraint, eval.directionConstraint,
                                         weight / accumulatedDirectionWeight);
    }
  }

  const auto &dc = global.directionConstraint;

  const float kPosition = distanceInclusion(global.sdBoundary, global.sdGuardBand);
  const float kYaw = angleInclusion(yawDelta(dc.yawCenter, viewingParams.yaw), dc.yawRange,
                                    dc.guardBandDirectionSize.value_or(0.F));
  const float kPitch = angleInclusion(viewingParams.pitch - dc.pitchCenter, dc.pitchRange,
                                      dc.guardBandDirectionSize.value_or(0.F));
  const float result = kPosition * kYaw * kPitch;
  assert(inRange(result, 0.F, 1.F));
  return result;
}

} // namespace TMIV::ViewingSpace
