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

#ifndef _TMIV_ATLASCONSTRUCTOR_ATLASCONSTRUCTOR_H_
#define _TMIV_ATLASCONSTRUCTOR_ATLASCONSTRUCTOR_H_

#include <deque>
#include <memory>

#include <TMIV/AtlasConstructor/IAggregator.h>
#include <TMIV/AtlasConstructor/IAtlasConstructor.h>
#include <TMIV/AtlasConstructor/IPacker.h>
#include <TMIV/AtlasConstructor/IPruner.h>
#include <TMIV/Common/Json.h>

namespace TMIV::AtlasConstructor {
// The AtlasConstructor of TMIV 1.0 provided by Technicolor
class AtlasConstructor : public IAtlasConstructor {
public:
  AtlasConstructor(const Common::Json & /*rootNode*/,
                   const Common::Json & /*componentNode*/);
  AtlasConstructor(const AtlasConstructor &) = delete;
  AtlasConstructor(AtlasConstructor &&) = default;
  AtlasConstructor &operator=(const AtlasConstructor &) = delete;
  AtlasConstructor &operator=(AtlasConstructor &&) = default;
  ~AtlasConstructor() override = default;

  using Vec2i = Common::Vec2i;
  using AtlasParameters = Metadata::AtlasParameters;

  void prepareIntraPeriod(CameraParametersList basicCameras,
                          CameraParametersList additionalCameras) override;
  void pushFrame(MVD16Frame basicViews, MVD16Frame additionalViews) override;
  void completeIntraPeriod() override;

  std::vector<Common::Vec2i> getAtlasSize() const override;
  const CameraParametersList &getCameraList() const override {
    return m_cameras;
  }
  const AtlasParametersList &getPatchList() const override {
    return m_patchList;
  }
  MVD16Frame popAtlas() override;

private:
  void writePatchInAtlas(const AtlasParameters &patch, const MVD16Frame &views,
                         MVD16Frame &atlas);

private:
  std::uint16_t m_nbAtlas = 0;
  Vec2i m_atlasSize = {1920, 1080};
  std::unique_ptr<IPruner> m_pruner;
  std::unique_ptr<IAggregator> m_aggregator;
  std::unique_ptr<IPacker> m_packer;
  std::vector<std::uint8_t> m_isReferenceView;
  std::vector<MVD16Frame> m_viewBuffer;
  Metadata::CameraParametersList m_cameras;
  Metadata::AtlasParametersList m_patchList;
  std::deque<MVD16Frame> m_atlasBuffer;
};
} // namespace TMIV::AtlasConstructor

#endif
