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

#include <TMIV/Renderer/ViewingSpaceController.h>

#include <cmath>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::Renderer {

namespace {

	float computeIndex(const ViewParams& metadata) {
		//TO DO
		return 1.F;
	}

	template <typename YUVD> 
	void inplaceFading_impl(YUVD& yuvd, const ViewParams& meta, float index) {
		//TO DO
	}


}

ViewingSpaceController::ViewingSpaceController(const Json & /*rootNode*/, const Json & /*componentNode*/) {
	//should we do something here ?
}

void ViewingSpaceController::inplaceFading(Texture444Depth10Frame &viewport, 
	                                       const ViewParams &metadata) const {
  // 1) computeIndex function call 
  float index = computeIndex(metadata);
  // 2) fading function call 
  inplaceFading_impl(viewport, metadata, index);  
}

void ViewingSpaceController::inplaceFading(Texture444Depth16Frame &viewport,
                                           const ViewParams &metadata) const {
  // 1) computeIndex function call
  float index = computeIndex(metadata);
  // 2) fading function call
  inplaceFading_impl(viewport, metadata, index);
}

} // namespace TMIV::Renderer
