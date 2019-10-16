#!/usr/bin/env python3

import json
import os

def loadJson(path):
    with open(path) as stream:
        return json.load(stream)
    raise 'Failed to open JSON'   

def saveJson(data, path):
    with open(path, 'w') as stream:
        json.dump(data, stream, indent=4, separators=(',', ': '))
        stream.write('\n')

def getSequenceParameters(seqId):
	sourceDepthBitDepth = 16
	outputCameraName = "v1"
	
	if seqId == "A":
		name = "ClassroomVideo"		
		startFrame = 23
		sourceCameraNames = [ "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9", "v10", "v11", "v12", "v13", "v14" ]
		width = 4096
		height = 2048
		maxLumaSamplesPerFrame = 33554432
	elif seqId == "B":
		name = "TechnicolorMuseum"		
		startFrame = 100
		sourceCameraNames = [ "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9", "v10", "v11", "v12", "v13", "v14", "v15", "v16", "v17",  "v18", "v19", "v20", "v21", "v22", "v23" ]
		width = 2048
		height = 2048
		maxLumaSamplesPerFrame = 33554432
	elif seqId == "C":
		name = "TechnicolorHijack"		
		startFrame = 0
		sourceCameraNames = [ "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9" ]
		width = 4096
		height = 4096
		maxLumaSamplesPerFrame = 100663296
	elif seqId == "D":
		name = "TechnicolorPainter"		
		startFrame = 10
		sourceCameraNames = [ "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9", "v10", "v11", "v12", "v13", "v14", "v15" ]
		width = 2048
		height = 1088
		maxLumaSamplesPerFrame = 22282240
	elif seqId == "E":
		name = "IntelFrog"		
		startFrame = 135
		sourceCameraNames = [ "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9", "v10", "v11", "v12", "v13" ]
		width = 1920
		height = 1080
		maxLumaSamplesPerFrame = 41472000
	elif seqId == "J":
		name = "OrangeKitchen"		
		startFrame = 0
		sourceCameraNames = [ "v00", "v01", "v02", "v03", "v04", "v05", "v06", "v07", "v08", "v09", "v10", "v11", "v12", "v13", "v14", "v15", "v16", "v17",  "v18", "v19", "v20", "v21", "v22", "v23", "v24" ]
		width = 1920
		height = 1080
		maxLumaSamplesPerFrame = 12441600
		sourceDepthBitDepth = 10
		outputCameraName = "v01"
	elif seqId == "L":
		name = "PoznanFencing"		
		startFrame = 30
		sourceCameraNames = [ "v00", "v01", "v02", "v03", "v04", "v05", "v06", "v07", "v08", "v09" ]
		width = 1920
		height = 1080
		maxLumaSamplesPerFrame = 16588800
		sourceDepthBitDepth = 8
		outputCameraName = "v01"
	else:
		exit(-1)
	return name, startFrame, sourceDepthBitDepth, sourceCameraNames, width, height, maxLumaSamplesPerFrame, outputCameraName
		
def createCfg(seqId, name, startFrame, sourceDepthBitDepth, sourceCameraNames, width, height, maxLumaSamplesPerFrame, outputCameraName, testPoint):
	
	if sourceDepthBitDepth > 8:
		depthFmt = "{}le".format(sourceDepthBitDepth)
	else:
		depthFmt = ""
	
	jsonData = {
		"startFrame": startFrame,
		"numberOfFrames": 97,
		"intraPeriod": 32,
		"SourceTexturePathFmt": "%s_texture_{}x{}_yuv420p10le.yuv".format(width,height),
		"SourceDepthPathFmt": "%s_depth_{}x{}_yuv420p{}.yuv".format(width,height,depthFmt),
		"SourceDepthBitDepth": sourceDepthBitDepth,
		"SourceCameraParameters": "{}.json".format(name),
		"SourceCameraNames": sourceCameraNames,
	    "AtlasTexturePathFmt": "ATL_S{}_{}_Tt_c%02d_{}x{}_yuv420p10le.yuv".format(seqId,testPoint,width,height),
		"AtlasDepthPathFmt": "ATL_S{}_{}_Td_c%02d_{}x{}_yuv420p10le.yuv".format(seqId,testPoint,width,height),
		"AtlasMetadataPath": "ATL_S{}_{}_Tm_c00.bit".format(seqId,testPoint),
		"OutputTexturePath": "TM1_S{}_{}_Tt_%s_{}x{}_yuv420p10le.yuv".format(seqId,testPoint,width,height),
		"OutputCameraName": outputCameraName,
		"OmafV1CompatibleFlag": False,
		"EncoderMethod": "Encoder",
		"Encoder": {
			"ViewOptimizerMethod": "ViewReducer",
			"ViewReducer": {},
			"AtlasConstructorMethod": "AtlasConstructor",
			"AtlasConstructor": {
				"PrunerMethod": "Pruner",
				"Pruner": {
					"RedundancyFactor": 0.05,
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
					width,
					height
				],
				"MaxLumaSamplesPerFrame": maxLumaSamplesPerFrame
			}
		},
		"DecoderMethod": "Decoder",
		"Decoder": {
			"AtlasDeconstructorMethod": "AtlasDeconstructor",
			"AtlasDeconstructor": {},
			"RendererMethod": "Renderer",
			"Renderer": {
				"SynthesizerMethod": "Synthesizer",
				"Synthesizer": {
					"rayAngleParameter": 20,
					"depthParameter": 20,
					"stretchingParameter": 0.6,
					"maxStretching": 5
				},
				"InpainterMethod": "Inpainter",
				"Inpainter": {}
			}
		}
	}
	return jsonData
		
		
if __name__ == '__main__':
	
	for testPoint in ["R0", "QP1", "QP2", "QP3", "QP4", "QP5"]:
		for seqId in ["A", "B", "C", "D", "E", "J", "L"]:
			name, start, bitDepth, cams, width, height, maxSamples, outputCameraName = getSequenceParameters(seqId)
			jsonData = createCfg(seqId, name, start, bitDepth, cams, width, height, maxSamples, outputCameraName, testPoint)	
			
			if testPoint == "R0":
				outDir = ""
			else:
				outDir = testPoint + "/"
				if not os.path.exists(testPoint):
					os.makedirs(testPoint)
			saveJson(jsonData, "{}TMIV_anchor_{}.json".format(outDir,seqId) )
	