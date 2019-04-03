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

#include <TMIV/AtlasDeconstructor/AtlasDeconstructor.h>
#include <TMIV/Common/Factory.h>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::AtlasDeconstructor {

AtlasDeconstructor::AtlasDeconstructor(const Common::Json &)
{
}

MVDFrame AtlasDeconstructor::getPatchFrameListFromAtlas(const PatchParameterList& patchList, const MVDFrame& atlas)
{
	MVDFrame patchFrameList;
	
	for(const auto& patch: patchList)
		patchFrameList.push_back(readPatchFromAtlas(patch, atlas));
	
	return patchFrameList;
}

TextureDepthFrame AtlasDeconstructor::readPatchFromAtlas(const PatchParameters& patch, const MVDFrame& atlas)
{
	TextureDepthFrame patchFrame;
	const TextureDepthFrame& currentAtlas = atlas[patch.atlasId];
	
	auto& texturePatchFrame = patchFrame.first;
	auto& depthPatchFrame = patchFrame.second;

	const auto& textureAtlasMap = currentAtlas.first;
	const auto& depthAtlasMap = currentAtlas.second;
	
	int w = patch.patchSize.x(), h = patch.patchSize.y();
	int xP = patch.patchPackingPos.y(), yP = patch.patchPackingPos.x();

	if(patch.patchRotation == Metadata::PatchRotation::upright)
	{
		for(int dy=0;dy<h;dy++)
		{
			// Y
			std::copy(
				textureAtlasMap.getPlane(0).row_begin(yP + dy) + xP,
				textureAtlasMap.getPlane(0).row_begin(yP + dy) + (xP + w),
				texturePatchFrame.getPlane(0).row_begin(dy)
			);
			
			// UV
			if((dy % 2) == 0)
			{
				for(int p=1;p<3;p++)
				{
					std::copy(
						textureAtlasMap.getPlane(p).row_begin((yP + dy) / 2) + xP / 2,
						textureAtlasMap.getPlane(p).row_begin((yP + dy) / 2) + (xP + w) / 2,
						texturePatchFrame.getPlane(p).row_begin(dy / 2)
					);
				}
			}
			
			// Depth
			std::copy(
				depthAtlasMap.getPlane(0).row_begin(yP + dy) + xP,
				depthAtlasMap.getPlane(0).row_begin(yP + dy) + (xP + w),
				depthPatchFrame.getPlane(0).row_begin(dy)
			);
		}
	}
	else
	{
		for(int dy=0;dy<h;dy++)
		{
			// Y
			std::copy(
				textureAtlasMap.getPlane(0).col_begin(xP + dy) + yP,
				textureAtlasMap.getPlane(0).col_begin(xP + dy) + (yP + w),
				std::make_reverse_iterator(texturePatchFrame.getPlane(0).row_end(dy))
			);
			
			// UV
			if((dy % 2) == 0)
			{
				for(int p=1;p<3;p++)
				{
					std::copy(
						textureAtlasMap.getPlane(0).col_begin((xP + dy) / 2) + (yP / 2),
						textureAtlasMap.getPlane(0).col_begin((xP + dy) / 2) + (yP + w) / 2,
						std::make_reverse_iterator(texturePatchFrame.getPlane(0).row_end(dy / 2))
					);
				}
			}
			
			// Depth
			std::copy(
				depthAtlasMap.getPlane(0).col_begin(xP + dy) + yP,
				depthAtlasMap.getPlane(0).col_begin(xP + dy) + (yP + w),
				std::make_reverse_iterator(depthPatchFrame.getPlane(0).row_end(dy))
			);
		}
	}
	
	return patchFrame;
}

} // namespace TMIV::AtlasDeconstructor
