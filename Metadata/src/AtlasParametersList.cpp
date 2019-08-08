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

#include <TMIV/Metadata/AtlasParametersList.h>
#include <sstream>

namespace TMIV::Metadata {
bool AtlasParameters::operator==(const AtlasParameters &other) const {
  return atlasId == other.atlasId && viewId == other.viewId &&
         patchSize == other.patchSize && posInView == other.posInView &&
         posInAtlas == other.posInAtlas && rotation == other.rotation;
}


Vec2i viewToAtlas(Vec2i viewPosition,
                                    const AtlasParameters &patch) {

  int w = patch.patchSize.x(), h = patch.patchSize.y();
  int xM = patch.posInView.x(), yM = patch.posInView.y();
  int xP = patch.posInAtlas.x(), yP = patch.posInAtlas.y();
  int x = viewPosition.x(), y = viewPosition.y();

  Vec2i pAtlas;

  if (patch.flip == Metadata::PatchFlip::none) {
    switch (patch.rotation) {
    case TMIV::Metadata::PatchRotation::upright:
      pAtlas.x() = x - xM + xP;
      pAtlas.y() = y - yM + yP;
      break;
    case TMIV::Metadata::PatchRotation::ccw:
      pAtlas.x() = y - yM + xP;
      pAtlas.y() = -x + xM + yP + w - 1;
      break;
    case TMIV::Metadata::PatchRotation::ht:
      pAtlas.x() = -x + xM + xP + w - 1;
      pAtlas.y() = -y + yM + yP + h - 1;
      break;
    case TMIV::Metadata::PatchRotation::cw:
      pAtlas.x() = -y + yM + xP + h - 1;
      pAtlas.y() = x - xM + yP;
      break;
    }
  } else { // patch.flip == Metadata::PatchFlip::vflip
    switch (patch.rotation) {
    case TMIV::Metadata::PatchRotation::upright:
      pAtlas.x() = x - xM + xP;
      pAtlas.y() = -y + yM + yP + h - 1;
      break;
    case TMIV::Metadata::PatchRotation::ccw:
      pAtlas.x() = y - yM + xP;
      pAtlas.y() = x - xM + yP;
      break;
    case TMIV::Metadata::PatchRotation::ht:
      pAtlas.x() = -x + xM + xP + w - 1;
      pAtlas.y() = y - yM + yP;
      break;
    case TMIV::Metadata::PatchRotation::cw:
      pAtlas.x() = -y + yM + xP + h - 1;
      pAtlas.y() = -x + xM + yP + w - 1;
      break;
    }
  }
  return pAtlas;
}

Vec2i atlasToView(Vec2i atlasPosition,
                                      const AtlasParameters &patch) {

  int w = patch.patchSize.x(), h = patch.patchSize.y();
  int xM = patch.posInView.x(), yM = patch.posInView.y();
  int xP = patch.posInAtlas.x(), yP = patch.posInAtlas.y();
  int x = atlasPosition.x(), y = atlasPosition.y();

  Vec2i pView;

  if (patch.flip == Metadata::PatchFlip::none) {
    switch (patch.rotation) {
    case Metadata::PatchRotation::upright:
      pView.x() = x - xP + xM;
      pView.y() = y - yP + yM;
      break;
    case Metadata::PatchRotation::ccw:
      pView.x() = -y + yP + xM + w - 1;
      pView.y() = x - xP + yM;
      break;
    case Metadata::PatchRotation::ht:
      pView.x() = -x + xP + xM + w - 1;
      pView.y() = -y + yP + yM + h - 1;
      break;
    case Metadata::PatchRotation::cw:
      pView.x() = y - yP + xM;
      pView.y() = -x + xP + yM + h - 1;
      break;
    }
  } else { // patch.flip == Metadata::PatchFlip::vflip
    switch (patch.rotation) {
    case Metadata::PatchRotation::upright:
      pView.x() = x - xP + xM;
      pView.y() = -y + yP + yM + h - 1;
      break;
    case Metadata::PatchRotation::ccw:
      pView.x() = y - yP + xM;
      pView.y() = x - xP + yM;
      break;
    case Metadata::PatchRotation::ht:
      pView.x() = -x + xP + xM + w - 1;
      pView.y() = y - yP + yM;
      break;
    case Metadata::PatchRotation::cw:
      pView.x() = -y + yP + xM + w - 1;
      pView.y() = -x + xP + yM + h - 1;
      break;
    }
  }
  return pView;
}

} // namespace TMIV::Metadata
