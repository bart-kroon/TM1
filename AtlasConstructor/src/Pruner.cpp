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

#include <TMIV/AtlasConstructor/Pruner.h>

namespace TMIV::AtlasConstructor {

Pruner::Pruner(const Common::Json& node)
{
	if(auto subnode = node.optional("RedundancyFactor"))
		m_redundancyFactor = subnode.asFloat();
	
	if(auto subnode = node.optional("ErosionIter"))
		m_erosionIter = subnode.asInt();
	
	if(auto subnode = node.optional("DilationIter"))
		m_dilationIter = subnode.asInt();
}

MaskList Pruner::doPruning(const Metadata::CameraParameterList& cameras, const Common::MVDFrame& views, const std::vector<std::uint8_t>& shouldNotBePruned)
{
	using namespace TMIV::Common;
	
	// Sort cameras for pruning
	std::vector<int> cameraOrderId(cameras.size());
	
	std::iota(cameraOrderId.begin(), cameraOrderId.end(), 0);
	
	std::sort(
		cameraOrderId.begin(),
		cameraOrderId.end(),
		[&shouldNotBePruned](int i1, int i2)
		{
			if(shouldNotBePruned[i1] != shouldNotBePruned[i2])
				return (shouldNotBePruned[i1] != 0);
			else
				return (i1 < i2);
		}
	);
	
	// Pruning loop
	MaskList masks(views.size());
	
	for(int id1=0;id1<views.size();id1++)
	{
		int viewToPruneId = cameraOrderId[id1];
		auto& currentMask = masks[viewToPruneId];

		currentMask.resize(views[viewToPruneId].first.getHeight(), views[viewToPruneId].first.getWidth());
		std::fill(currentMask.begin(), currentMask.end(), 1);
		
		if(!shouldNotBePruned[viewToPruneId])
		{
			// Depth-based redundancy removal
			Mat<float> depthMapReference; // TODO convert source depth map in float

			for(int id2=0;id2<id1;id2++)
			{
				int viewPrunedId = cameraOrderId[id2];
				Mat<float> depthMapSynthesized; // TODO synthesize depth of view[viewToPruneId] from view[viewPrunedId] + convert synthesized map in float
					
				for(int k=0;k<depthMapReference.size();k++)
				{
					auto& mask = currentMask[k];
					
					if(0 < mask)
					{
						float zRef = depthMapReference[k];
						float zSynth = depthMapSynthesized[k];
						
						if((0. < zSynth) && (fabs(zSynth - zRef) < m_redundancyFactor * zSynth))
							mask = 0;
					}
				}
			}
			
			// Mask post-processing
			int w = currentMask.n(), h = currentMask.m();
			int wLast = w - 1, hLast = h - 1;
			std::array<int, 8> neighbourOffset = { -1 - w, -w, 1 - w, -1, 1, -1 + w, w, 1 + w }; 
			Mask otherMask(currentMask.sizes());

				// Erosion
			if(0 < m_erosionIter)
			{
				Mask& inputMask = (m_erosionIter % 2) ? otherMask : currentMask;
				Mask& outputMask = (m_erosionIter % 2) ? currentMask : otherMask;
				
				inputMask = currentMask;

				for(int erosionId=0;erosionId<m_erosionIter;erosionId++)
				{
					for(int y=1,k1=w+1;y<hLast;y++,k1+=w)
					{
						for(int x=1,k2=k1;x<wLast;x++,k2++)
						{
							auto maskIn = inputMask[k2];
							auto& maskOut = outputMask[k2];
							
							maskOut = maskIn;
						
							if(0 < maskIn)
							{
								for(auto o: neighbourOffset)
								{
									if(inputMask[k2+o] == 0)
									{
										maskOut = 0;
										break;
									}
								}
							}
						}
					}
					
					std::swap(inputMask, outputMask);
				}
			}
			
				// Dilation
			if(0 < m_dilationIter)
			{
				Mask& inputMask = (m_dilationIter % 2) ? otherMask : currentMask;
				Mask& outputMask = (m_dilationIter % 2) ? currentMask : otherMask;
				
				inputMask = currentMask;

				for(int dilationId=0;dilationId<m_dilationIter;dilationId++)
				{
					for(int y=1,k1=w+1;y<hLast;y++,k1+=w)
					{
						for(int x=1,k2=k1;x<wLast;x++,k2++)
						{
							auto maskIn = inputMask[k2];
							auto& maskOut = outputMask[k2];
							
							maskOut = maskIn;
						
							if(0 < maskIn)
							{
								for(auto o: neighbourOffset)
								{
									if(0 < inputMask[k2+o])
									{
										maskOut = 1;
										break;
									}
								}
							}
						}
					}
					
					std::swap(inputMask, outputMask);
				}
			}
		}
	}
	
	return masks;
}

} // namespace TMIV::AtlasConstructor
