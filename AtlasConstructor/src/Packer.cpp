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

#include <TMIV/AtlasConstructor/Packer.h>
#include "MaxRectPiP.h"

namespace TMIV::AtlasConstructor {

#if 0
	////////////////////////////////////////////////////////////////////////////////
omaf::Patch getPatchFromCluster(const Cluster& c)
{
	omaf::Patch p;

	p.setProjectionId(c.getProjectionId());
	p.setMappingPosition({ (short) c.jmin(), (short) c.imin() });
	p.setMappingSize({ (short) c.width(), (short) c.height() });

	return p;
}

////////////////////////////////////////////////////////////////////////////////
std::pair< std::vector<omaf::Patch>, std::vector<int> > packClusters(const std::vector< std::unique_ptr<omaf::projection::Base> >& projection, const gpu::uVec2& atlasSize, const std::vector<Cluster>& clusterList, const std::vector< std::vector< std::vector<ushort> > >& clusteringBuffer, const std::vector< std::vector<gpu::core::Texture2D> >& clusteringMap, const std::vector<uint>& clusterMinFilling, uint packingAlignment, bool pip, uint overlap)
{
	std::pair< std::vector<omaf::Patch>, std::vector<int> > packingOutput;
	std::vector<omaf::Patch>& patchList = packingOutput.first;
	std::vector<int>& clusterIds = packingOutput.second;
	Packer::Output packerOutput;
	uint packed = 0, discarded = 0;

	// Initialization
	Packer packer(atlasSize.x(), atlasSize.y(), packingAlignment, pip);

	if(0 < clusterList[0].getFilling())
	{
		Cluster c0 = Cluster::align(clusterList[0], packingAlignment);
		
		patchList.push_back(getPatchFromCluster(c0));
		clusterIds.push_back(0);
		
		packer.initialize(c0, clusteringMap[0][0], packerOutput);
		
		patchList[0].setPackingPosition({ (short) packerOutput.x(), (short) packerOutput.y() });
		patchList[0].setPackingRotation(packerOutput.isRotated());
		
		packed += c0.getFilling();
	}
	else
	{
		patchList.push_back(Patch());
		clusterIds.push_back(0);
	}

	// Packing loop
	auto comp = [](const Cluster& p1, const Cluster& p2) { return (p1.getArea() < p2.getArea()); };
	std::priority_queue<Cluster, std::vector<Cluster>, decltype(comp)> clusterToPack(comp);
	
	for(uint i=1;i<clusterList.size();i++)
		clusterToPack.push(clusterList[i]);

	while(!clusterToPack.empty() && (patchList.size() < PatchNumberLimit))
	{
		const Cluster& cluster = clusterToPack.top();

		if(packer.push(cluster, clusteringMap[cluster.getProjectionId()][cluster.getPeelingId()], packerOutput))
		{
			omaf::Patch p = getPatchFromCluster(cluster);
			
			p.setPackingPosition({ (short) packerOutput.x(), (short) packerOutput.y() });
			p.setPackingRotation(packerOutput.isRotated());

			patchList.push_back(std::move(p));
			clusterIds.push_back(cluster.getClusterId());
			
			packed += cluster.getFilling();
		}
		else
		{
			std::pair<Cluster, Cluster> cc = splitCluster(cluster, projection[cluster.getProjectionId()]->size(), clusteringBuffer[cluster.getProjectionId()][cluster.getPeelingId()], overlap);
			
			if(clusterMinFilling[cluster.getProjectionId()] <= cc.first.getFilling())
				clusterToPack.push(std::move(cc.first));
			else
				discarded += cc.first.getFilling();
			
			if(clusterMinFilling[cluster.getProjectionId()] <= cc.second.getFilling())
				clusterToPack.push(std::move(cc.second));
			else
				discarded += cc.second.getFilling();
		}

		clusterToPack.pop();
	}
	
	LOG_INFO("Packing Filling Ratio: " + float2str(100.f * float(packed) / (atlasSize.x() * atlasSize.y()), 2, 2) + "% ("+ any2str(packed) + " packed / " + any2str(discarded) + " discarded)");

	return packingOutput;
}
#endif


////////////////////////////////////////////////////////////////////
Packer::Packer(const Common::Json& node)
{
	// TODO
}
  
Metadata::PatchParameterList Packer::doPacking(const MaskList& masks, const std::vector<std::uint8_t>& shouldNotBeSplit)
{
	// Mask clustering
	ClusterList clusterList;
	ClusteringMapList clusteringMap;
	
	for(int cameraId=0;cameraId<masks.size();cameraId++)
	{
		auto clusteringOutput = Cluster::retrieve(cameraId, masks[cameraId], clusterList.size(), shouldNotBeSplit[cameraId]);
		
		std::move(clusteringOutput.first.begin(), clusteringOutput.first.end(), std::back_inserter(clusterList));
		clusteringMap.push_back(std::move(clusteringOutput.second));
	}

	// Packing
	// TODO
	return Metadata::PatchParameterList();
	
	
}

} // namespace TMIV::AtlasConstructor
