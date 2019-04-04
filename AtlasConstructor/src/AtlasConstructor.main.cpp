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
#include <TMIV/AtlasConstructor/Pruner.h>
#include <TMIV/AtlasConstructor/Aggregator.h>
#include <TMIV/AtlasConstructor/Packer.h>
#include <TMIV/Common/Common.h>
#include <TMIV/Common/Application.h>
#include <TMIV/Common/Factory.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::AtlasConstructor
{

class Application : public Common::Application
{
private:
	class ComponentRegistrator
	{
	public:
		ComponentRegistrator()
		{
			Factory<IAtlasConstructor>::getInstance().registerAs<AtlasConstructor>("AtlasConstructor");
			Factory<IPruner>::getInstance().registerAs<Pruner>("Pruner");
			Factory<IAggregator>::getInstance().registerAs<Aggregator>("Aggregator");
			Factory<IPacker>::getInstance().registerAs<Packer>("Packer");
		}
	};
private:
	static ComponentRegistrator m_componentRegistrator;
	int m_startFrame = 0;
	int m_numberOfFrames = 32;
	int m_intraPeriod = 32;
	Vec2i m_sourceResolution = { 0, 0 };
	std::vector<int> m_baseViewId = { 0 };
	std::vector<int> m_additionalViewId = { 1 };
	unique_ptr<IAtlasConstructor> m_atlasContructor;
public:
	Application(vector<const char *> argv): Common::Application{"AtlasConstructor", move(argv)}
	{
		m_startFrame = json().require("startFrame").asInt();
		m_numberOfFrames = json().require("numberOfFrames").asInt();
		m_intraPeriod = json().require("intraPeriod").asInt();
		m_sourceResolution = json().require("SourceResolution").asIntVector<2>();
		
		if (auto subnode = json().optional("BaseView"))
		{
			m_baseViewId.clear();

			for(auto i=0u; i < subnode.size() ; i++)
			m_baseViewId.push_back(subnode.at(i).asInt());
		}
		
		if (auto subnode = json().optional("AdditionalView"))
		{
			m_additionalViewId.clear();
			
			for(auto i=0u; i < subnode.size() ; i++)
				m_additionalViewId.push_back(subnode.at(i).asInt());
		}
		
		m_atlasContructor = create<IAtlasConstructor>("AtlasConstructor");
	}
	
	void run() override
	{
		// Loading cameras
		auto allCameras = loadCameras();
		CameraParameterList baseCameras, additionalCameras;
		
		for(auto id: m_baseViewId)
			baseCameras.push_back(std::move(allCameras[id]));
		
		for(auto id: m_additionalViewId)
			additionalCameras.push_back(std::move(allCameras[id]));
		
		
		for (int intraFrame = 0; intraFrame < m_numberOfFrames; intraFrame += m_intraPeriod)
		{
			int endFrame = min(m_numberOfFrames, intraFrame + m_intraPeriod);
			cout << "Intra period: [" << intraFrame << ", " << endFrame << ")\n";

			m_atlasContructor->prepareIntraPeriod();
			for (int frame = intraFrame; frame < endFrame; ++frame)
			{
				MVDFrame allViews = loadViews(m_startFrame + frame, allCameras.size());
				
				// Lazy view optimization
				MVDFrame baseViews, additionalViews;
				CameraParameterList baseCameras, additionalCameras;
				
				for(auto id: m_baseViewId)
				{
					baseViews.push_back(std::move(allViews[id]));
					baseCameras.push_back(allCameras[id]);
				}
				
				for(auto id: m_additionalViewId)
				{
					additionalViews.push_back(std::move(allViews[id]));
					additionalCameras.push_back(allCameras[id]);
				}

				// Push frame
				cout << "Push input frame " << (m_startFrame + frame) << '\n';
				m_atlasContructor->pushFrame(baseCameras, baseViews, additionalCameras, additionalViews);
			}
			
			m_atlasContructor->completeIntraPeriod();

			saveMetadata(intraFrame, m_atlasContructor->getCameras(),
						m_atlasContructor->getPatchList());

			for (int frame = intraFrame; frame < endFrame; ++frame)
			{
				cout << "Pop output atlas " << frame << '\n';
				auto atlases = m_atlasContructor->popAtlas();
				saveViews(frame, atlases);
			}
		}
	}
private:
	Metadata::CameraParameterList loadCameras() const
	{
		ifstream stream{json().require("SourceCameraParameters").asString()};
		
		if (!stream.good())
		{
			throw runtime_error("Failed to load source camera parameters");
		}
		
		return Metadata::loadCamerasFromJson(Common::Json{stream}.require("cameras"), json().require("SourceCameraNames").asStringVector());
	}
	MVDFrame loadViews(int inputFrame, size_t numberOfViews) const
	{
		MVDFrame result(numberOfViews);

		for (auto view = 0u; view < numberOfViews; ++view)
		{
			auto id = json().require("SourceCameraIDs").at(view).asInt();
			
			{
				auto texturePath = Common::format(json().require("SourceTexturePathFmt").asString().c_str(), id);
				ifstream stream{texturePath};
				
				stream.seekg(streampos(inputFrame) * m_sourceResolution.x() * m_sourceResolution.y() * 3); // YUV420P10
				result[view].first.resize(m_sourceResolution.y(), m_sourceResolution.x());
				result[view].first.read(stream);
			}
			
			{
				auto depthPath = Common::format(json().require("SourceDepthPathFmt").asString().c_str(), id);
				ifstream stream{depthPath};
				
				stream.seekg(streampos(inputFrame) * m_sourceResolution.x() * m_sourceResolution.y() * 2); // YUV400P16
				result[view].second.resize(m_sourceResolution.y(), m_sourceResolution.x());
				result[view].second.read(stream);
			}
		}
		
		return result;
	}
	void saveViews(int outputFrame, const MVDFrame& atlases) const
	{
		// TODO
	}
	void saveMetadata(int outputFrame, const CameraParameterList &cameras, const PatchParameterList &patches)
	{
		// TODO
	}
};

} // namespace TMIV::AtlasConstructor

int main(int argc, char *argv[])
{
	TMIV::AtlasConstructor::Application app{{argv, argv + argc}};
	app.run();
	return 0;
}
