#!/usr/bin/env python3
#
# The copyright in this software is being made available under the BSD
# License, included below. This software may be subject to other third party
# and contributor rights, including patent rights, and no such rights are
# granted under this license.
#
# Copyright (c) 2010-2019, ISO/IEC
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the ISO/IEC nor the names of its contributors may
#    be used to endorse or promote products derived from this software without
#    specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.

import json
import os
import sys

def saveJson(data, path):
    with open(path, 'w') as stream:
        json.dump(data, stream, indent=4, separators=(',', ': '))
        stream.write('\n')

		
class ConfigurationParameters:
	def __init__(self, seqId, anchorType):
		self.sourceDepthBitDepth = 16
		self.outputCameraName = "v1"
				
		if anchorType == 'anchor97':
			self.numberOfFrames = 97
		else:
			self.numberOfFrames = 17
			
		if seqId == "A":
			self.name = "ClassroomVideo"		
			self.startFrame = 23
			self.sourceCameraNames = [ "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9", "v10", "v11", "v12", "v13", "v14" ]
			sourceCameraNamesViewAnchor = [ "v0", "v7", "v8", "v9", "v10", "v11", "v12", "v13", "v14" ]
			self.numberOfViewsPerPass = [2,4,9]	
			self.width = 4096
			self.height = 2048
			self.maxLumaSamplesPerFrame = 33554432
			maxLumaSamplesPerFrameViewAnchor = 150994944
		elif seqId == "B":
			self.name = "TechnicolorMuseum"		
			self.startFrame = 100
			self.sourceCameraNames = [ "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9", "v10", "v11", "v12", "v13", "v14", "v15", "v16", "v17",  "v18", "v19", "v20", "v21", "v22", "v23" ]
			sourceCameraNamesViewAnchor = [ "v0", "v1", "v4", "v8", "v11", "v12", "v13", "v17" ]
			self.numberOfViewsPerPass = [2,4,8]	
			self.width = 2048
			self.height = 2048
			self.maxLumaSamplesPerFrame = 33554432
			maxLumaSamplesPerFrameViewAnchor = 67108864
		elif seqId == "C":
			self.name = "TechnicolorHijack"		
			self.startFrame = 0
			self.sourceCameraNames = [ "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9" ]
			sourceCameraNamesViewAnchor = [ "v1", "v4", "v5", "v8", "v9" ]
			self.numberOfViewsPerPass = [2,4,5]	
			self.width = 4096
			self.height = 4096
			self.maxLumaSamplesPerFrame = 100663296
			maxLumaSamplesPerFrameViewAnchor = 167772160
		elif seqId == "D":
			self.name = "TechnicolorPainter"		
			self.startFrame = 10
			self.sourceCameraNames = [ "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9", "v10", "v11", "v12", "v13", "v14", "v15" ]
			sourceCameraNamesViewAnchor = [ "v0", "v3", "v5", "v6", "v9", "v10", "v12", "v15" ]
			self.numberOfViewsPerPass = [2,4,8]	
			self.width = 2048
			self.height = 1088
			self.maxLumaSamplesPerFrame = 22282240
			maxLumaSamplesPerFrameViewAnchor = 35651584
		elif seqId == "E":
			self.name = "IntelFrog"		
			self.startFrame = 135
			self.sourceCameraNames = [ "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9", "v10", "v11", "v12", "v13" ]
			sourceCameraNamesViewAnchor = [ "v1", "v3", "v5", "v7", "v9", "v11", "v13" ]
			self.numberOfViewsPerPass = [2,4,7]	
			self.width = 1920
			self.height = 1080
			self.maxLumaSamplesPerFrame = 41472000
			maxLumaSamplesPerFrameViewAnchor = 29030400
		elif seqId == "J":
			self.name = "OrangeKitchen"		
			self.startFrame = 0
			self.sourceCameraNames = [ "v00", "v01", "v02", "v03", "v04", "v05", "v06", "v07", "v08", "v09", "v10", "v11", "v12", "v13", "v14", "v15", "v16", "v17",  "v18", "v19", "v20", "v21", "v22", "v23", "v24" ]
			sourceCameraNamesViewAnchor = [ "v00", "v02", "v04", "v10", "v12", "v14", "v20", "v22", "v24" ]
			self.numberOfViewsPerPass = [2,4,7]	
			self.width = 1920
			self.height = 1080
			self.maxLumaSamplesPerFrame = 12441600
			maxLumaSamplesPerFrameViewAnchor = 37324800
			self.sourceDepthBitDepth = 10
			self.outputCameraName = "v01"
		elif seqId == "L":
			self.name = "PoznanFencing"		
			self.startFrame = 30
			self.sourceCameraNames = [ "v00", "v01", "v02", "v03", "v04", "v05", "v06", "v07", "v08", "v09" ]
			sourceCameraNamesViewAnchor = [ "v00", "v02", "v04", "v06", "v08" ]
			self.numberOfViewsPerPass = [2,4,7]	
			self.width = 1920
			self.height = 1080
			self.maxLumaSamplesPerFrame = 16588800
			maxLumaSamplesPerFrameViewAnchor = 20736000
			self.sourceDepthBitDepth = 8
			self.outputCameraName = "v01"
		else:
			print("Logic error: unknown sequence id")
			exit(-2)
		
		if anchorType == 'view_anchor17': 
			self.sourceCameraNames = sourceCameraNamesViewAnchor
			self.maxLumaSamplesPerFrame = maxLumaSamplesPerFrameViewAnchor


	
def makeConfiguration(anchorType, testPoint, seqId, parameters):
	p = parameters
	W = p.width
	H = p.height
	
	if p.sourceDepthBitDepth > 8:
		depthFmt = "{}le".format(p.sourceDepthBitDepth)
	else:
		depthFmt = ""
	
	synthesizer = { "rayAngleParameter": 20,
				"depthParameter": 20,
				"stretchingParameter": 0.6,
				"maxStretching": 5 }
	
	if anchorType == 'view_anchor17':
		decoder =  {
			"AtlasDeconstructorMethod": "AtlasDeconstructor",
			"AtlasDeconstructor": {},
			"RendererMethod": "MultipassRenderer",
			"MultipassRenderer": {
				"NumberOfPasses": 3,
				"NumberOfViewsPerPass": p.numberOfViewsPerPass,
				"SynthesizerMethod": "Synthesizer",
				"Synthesizer": synthesizer,
				"InpainterMethod": "Inpainter",
				"Inpainter": {}
			}
		}
		redundancyFactor = 0.02
		viewOptimizerMethod = "NoViewOptimizer"
	else:
		decoder =  {
			"AtlasDeconstructorMethod": "AtlasDeconstructor",
			"AtlasDeconstructor": {},
			"RendererMethod": "Renderer",
			"Renderer": {
				"SynthesizerMethod": "Synthesizer",
				"Synthesizer": synthesizer,
				"InpainterMethod": "Inpainter",
				"Inpainter": {}
			}
		}
		redundancyFactor = 0.05
		viewOptimizerMethod = "ViewReducer"
	
	
	config = {
		"startFrame": p.startFrame,
		"numberOfFrames": p.numberOfFrames,
		"intraPeriod": 32,
		"SourceTexturePathFmt": "%s_texture_{}x{}_yuv420p10le.yuv".format(W,H),
		"SourceDepthPathFmt": "%s_depth_{}x{}_yuv420p{}.yuv".format(W,H,depthFmt),
		"SourceDepthBitDepth": p.sourceDepthBitDepth,
		"SourceCameraParameters": "{}.json".format(p.name),
		"SourceCameraNames": p.sourceCameraNames,
	    "AtlasTexturePathFmt": "ATL_S{}_{}_Tt_c%02d_{}x{}_yuv420p10le.yuv".format(seqId,testPoint,W,H),
		"AtlasDepthPathFmt": "ATL_S{}_{}_Td_c%02d_{}x{}_yuv420p10le.yuv".format(seqId,testPoint,W,H),
		"AtlasMetadataPath": "ATL_S{}_{}_Tm_c00.bit".format(seqId,testPoint),
		"OutputTexturePath": "TM1_S{}_{}_Tt_%s_{}x{}_yuv420p10le.yuv".format(seqId,testPoint,W,H),
		"OutputCameraName": p.outputCameraName,
		"OmafV1CompatibleFlag": False,
		"EncoderMethod": "Encoder",
		"Encoder": {
			"ViewOptimizerMethod": viewOptimizerMethod,
			"{}".format(viewOptimizerMethod): {},
			"AtlasConstructorMethod": "AtlasConstructor",
			"AtlasConstructor": {
				"PrunerMethod": "Pruner",
				"Pruner": {
					"RedundancyFactor": redundancyFactor,
					"ErosionIter": 1,
					"DilationIter": 5
				},
				"AggregatorMethod": "Aggregator",
				"Aggregator": {},
				"PackerMethod": "Packer",
				"Packer": {
					"Alignment": 8,
					"MinPatchSize": 16,
					"Overlap": 1,
					"PiP": 1
				},
				"AtlasResolution": [
					W,
					H
				],
				"MaxLumaSamplesPerFrame": p.maxLumaSamplesPerFrame
			}
		},
		"DecoderMethod": "Decoder",
		"Decoder": decoder
	}
	return config
		
		
if __name__ == '__main__':

	if sys.version_info[0] < 3:
		print ("Error: Python version < 3")
		exit(-1)

	for testPoint in ["R0", "QP1", "QP2", "QP3", "QP4", "QP5"]:
		if not os.path.exists(testPoint):
			os.makedirs(testPoint)
		for anchorType in [ 'anchor97', 'anchor17', 'view_anchor17' ]:
			for seqId in ["A", "B", "C", "D", "E", "J", "L"]:
				print(anchorType, testPoint, seqId)
				parameters = ConfigurationParameters(seqId, anchorType)
				config = makeConfiguration(anchorType, testPoint, seqId, parameters)	
				saveJson(config, "{}/TMIV_{}_{}.json".format(testPoint, anchorType, seqId) )
	