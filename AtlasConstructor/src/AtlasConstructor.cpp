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

#include <TMIV/AtlasConstructor/AtlasConstructor.h>
#include <TMIV/Common/Factory.h>
#include "Cluster.h"

using namespace std;
using namespace TMIV::Common;

namespace TMIV::AtlasConstructor {

AtlasConstructor::AtlasConstructor(const Common::Json &node) {
  m_pruner = Factory<IPruner>::getInstance().create("Pruner", node);
  m_aggregator = Factory<IAggregator>::getInstance().create("Aggregator", node);
  m_packer = Factory<IPacker>::getInstance().create("Packer", node);
}

void AtlasConstructor::prepareIntraPeriod()
{
	m_viewBuffer.clear();
	m_aggregator->prepareIntraPeriod();
}

void AtlasConstructor::pushFrame(
				const CameraParameterList &baseCameras,
				const MVDFrame &baseViews,
				const CameraParameterList &additionalCameras,
				const MVDFrame &additionalViews)
{
	// Merging
	CameraParameterList cameras;
	MVDFrame views;
	std::vector<std::uint8_t> isReferenceView;
	
	cameras.insert(cameras.end(), baseCameras.begin(), baseCameras.end());
	views.insert(views.end(), baseViews.begin(), baseViews.end());
	isReferenceView.insert(isReferenceView.end(), isReferenceView.size(), 1);
	
	cameras.insert(cameras.end(), additionalCameras.begin(), additionalCameras.end());
	views.insert(views.end(), additionalViews.begin(), additionalViews.end());
	isReferenceView.insert(isReferenceView.end(), additionalViews.size(), 0);
	
	// Cameras definition
	if(m_viewBuffer.empty())
	{
		m_isReferenceView = std::move(isReferenceView);
		m_cameras = std::move(cameras);
	}

	// View Buffering
	m_viewBuffer.push_back(std::move(views));
	
	// Pruning mask
	MaskList masks = m_pruner->doPruning(cameras, views, m_isReferenceView);
	
	// Mask Aggregation
	m_aggregator->pushMask(masks);
}

void AtlasConstructor::completeIntraPeriod()
{
	// Aggregated mask
	m_aggregator->completeIntraPeriod();
	const MaskList& aggregatedMask = m_aggregator->getAggregatedMask();
	
	// Packing
	m_patchList = m_packer->doPacking(aggregatedMask, m_isReferenceView);
	
	// Atlas construction
	// TODO

}

Common::MVDFrame AtlasConstructor::popAtlas()
{
	
	// TODO
  return {};
}

} // namespace TMIV::AtlasConstructor
