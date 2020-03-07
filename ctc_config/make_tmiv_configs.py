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
#  * Neither the seqName of the ISO/IEC nor the names of its contributors may
#    be used to endorse or promote products derived from this software without
#    specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
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

def checkAlignment(size):
	if size % 8 != 0:
		raise 'The frame size {} is not aligned'.format(size)
	return size

class TestConfiguration:
	def __init__(self, sourceDir, anchorId, seqId, testPoint):
		self.__sourceDir = sourceDir
		self.__anchorId = anchorId
		self.__seqId = seqId
		self.__testPoint = testPoint
		
		with open(self.sequenceJsonPath(), 'r') as stream:			
			self.sequenceParams = json.load(stream)

	@property
	def sourceDir(self):
		return self.__sourceDir

	@property
	def anchorId(self):
		return self.__anchorId

	@property
	def seqId(self):
		return self.__seqId

	@property
	def testPoint(self):
		return self.__testPoint

	def ctcArchive(self):
		return self.sourceDir == '.'

	def numberOfFrames(self):
		if self.anchorId == 'A97' or self.anchorId == 'R97' or self.anchorId == 'E97':
			return 97
		return 17

	def startFrame(self):
		return {
			'A': 23,
			'B': 100,
			'C': 0,
			'D': 10,
			'E': 135,
			'J': 0,
			'L': 30,
			'N': 60,
			'P': 0,
			'U': 0,
			'T': 0
		}[self.seqId]		

	def seqName(self):
		return {
			'A': 'ClassroomVideo',
			'B': 'TechnicolorMuseum',
			'C': 'InterdigitalHijack',
			'D': 'TechnicolorPainter',
			'E': 'IntelFrog',
			'J': 'OrangeKitchen',
			'L': 'PoznanFencing',
			'N': 'NokiaChess',
			'P': 'PoznanCarpark',
			'U': 'PoznanStreet',
			'T': 'PoznanHall'
		}[self.seqId]

	def configPath(self):
		x = os.path.dirname(sys.argv[0])
		if len(x) > 0:
			return x
		return '.'

	def sourceDirectory(self):
		return self.sourceDir.format(self.seqId, self.seqName())

	def sequenceJsonPath(self):
		return os.path.realpath('{}/sequences/{}.json'.format(self.configPath(), self.seqName()))

	def sourceCameraParameters(self):
		if self.ctcArchive():
			return '{}.json'.format(self.seqName())	
		return self.sequenceJsonPath()

	def atlasTexturePathFmt(self):
		return 'ATL_S{}_{}_Tt_c{{:02}}_{{}}x{{}}_yuv420p10le.yuv'.format(self.seqId, self.testPoint)

	def atlasDepthPathFmt(self):
		return 'ATL_S{}_{}_Td_c{{:02}}_{{}}x{{}}_yuv420p10le.yuv'.format(self.seqId, self.testPoint)
		
	def atlasMetadataPath(self):
		return 'ATL_S{}_{}_Tm_c00.bit'.format(self.seqId, self.testPoint)

	def outputDirectory(self):
		if self.ctcArchive():
			return '.'
		return os.path.realpath(os.path.dirname(self.path()))

	def numberOfSourceViews(self):
		return {
			'A': 15,
			'B': 24,
			'C': 10,
			'D': 16,
			'E': 13,
			'J': 25,
			'L': 10,
			'N': 10,
			'P': 9,
			'U': 9,
			'T': 9
		}[self.seqId]

	def firstSourceView(self):
		if self.seqId == 'E':
			return 1
		return 0

	def viewNameFormat(self):
		if self.seqId == 'J' or self.seqId == 'L':
			return 'v{:02}'
		return 'v{}'

	def allSourceCameraNames(self):
		return list(map(lambda x: self.viewNameFormat().format(x + self.firstSourceView()), range(0, self.numberOfSourceViews())))
		
	def parameters(self):
		return {
			'numberOfFrames': self.numberOfFrames(),
			'SourceDirectory': self.sourceDirectory(),
			'SourceCameraParameters': self.sourceCameraParameters(),
			'AtlasTexturePathFmt': self.atlasTexturePathFmt(),
			'AtlasDepthPathFmt': self.atlasDepthPathFmt(),
			'AtlasMetadataPath': self.atlasMetadataPath()
		}

	def saveTmivJson(self):	
		os.makedirs(os.path.dirname(self.path()), exist_ok = True)
		with open(self.path(), 'w') as stream:
			json.dump(self.parameters(), stream, indent=4, sort_keys=True, separators=(',', ': '))
			stream.write('\n')

class DecoderConfiguration(TestConfiguration):
	def __init__(self, sourceDir, anchorId, seqId, testPoint):
		TestConfiguration.__init__(self, sourceDir, anchorId, seqId, testPoint)
		print("Decoder configuration: {} S{} {}".format(anchorId, seqId, testPoint))

	def outputTexturePath(self):
		return '{}_S{}_{}_Tt_{}_{{}}x{{}}_yuv420p10le.yuv'.format(
			self.anchorId, self.seqId, self.testPoint, self.outputCameraName)

	def ViewWeightingSynthesizer(self):
		return {
			'angularScaling': 1.5,
			'blendingFactor': 0.03,
			'filteringPass': 1,
			'minimalWeight': 2.5,
			'overloadFactor': 2.0,
			'stretchFactor': 100.0
		}
	
	def rendererMethod(self):
		return 'Renderer'

	def renderer(self):
		config = {
			'SynthesizerMethod': 'ViewWeightingSynthesizer',
			'ViewWeightingSynthesizer': self.ViewWeightingSynthesizer(),
			'InpainterMethod': 'Inpainter',
			'Inpainter': {},
			'ViewingSpaceControllerMethod': 'ViewingSpaceController',
			'ViewingSpaceController': {}
		}
		return config

	def poseTraceBasename(self):
		return '{}{}.csv'.format(self.seqId, self.outputCameraName)

	def poseTracePath(self):
		if self.ctcArchive():
			return self.poseTraceBasename()
		return os.path.realpath('{}/pose_traces/{}'.format(self.configPath(), self.poseTraceBasename()))

	def parameters(self):
		config = TestConfiguration.parameters(self)
		config.update({
			'OutputDirectory': self.outputDirectory(),
			'OutputTexturePath': self.outputTexturePath(),
			'OutputCameraName': self.outputCameraName,
			'DecoderMethod': 'Decoder',
			'Decoder': {
				'AtlasDeconstructorMethod': 'AtlasDeconstructor',
				'CullerMethod': 'SubBlockCuller',
				'SubBlockCuller': {},
				'RendererMethod': self.rendererMethod(),
				self.rendererMethod(): self.renderer(),
				'depthEdgeMagnitudeTh': 10,
				'maxCurvature': 5,
				'minForegroundConfidence': 0.5
			}
		})
		if self.outputCameraName[0] == 'p':
			config['OutputCameraName'] = 'viewport'
			config['PoseTracePath'] = self.poseTracePath()
			if self.anchorId == 'A97' or self.anchorId == 'R97' or self.anchorId == 'E97':
				config['extraNumberOfFrames'] = 300 - 97
		return config

	def testPointUnlessR0(self):
		if self.testPoint == 'R0':
			return '.'
		return self.testPoint

	def path(self):
		return '{0}/S{1}/{2}/TMIV_{0}_S{1}_{3}_{4}.json'.format(
			self.anchorId, self.seqId, self.testPointUnlessR0(), self.testPoint, self.outputCameraName)

	def wspsnrPath(self):
		return '{0}/S{1}/{2}/WSPSNR_{0}_S{1}_{3}_{4}.json'.format(
			self.anchorId, self.seqId, self.testPointUnlessR0(), self.testPoint, self.outputCameraName)

	def originalFilePath(self, camera):
		path = '{}_texture_{}x{}_yuv420p10le.yuv'.format(camera['Name'], camera['Resolution'][0], camera['Resolution'][1])
		if self.sourceDirectory() == '.':
			return path
		return os.path.join(self.sourceDirectory(), path)

	def outputCamera(self):
		target = self.outputCameraName
		for camera in self.sequenceParams['cameras']:
			if camera['Name'] == target:
				return camera
		raise 'Could not find output camera in sequence configuration'

	def wspsnrParameters(self):
		camera = self.outputCamera()
		config = {
			"Version": "2.0",
			"Projection": camera['Projection'],
			"Original_file_path": self.originalFilePath(camera),
			"Reconstructed_file_path": self.outputTexturePath(),
			"ColorSpace": "YUV420",
			"Video_width": camera['Resolution'][0],
			"Video_height": camera['Resolution'][1],
			"BitDepth": 10,
			"Start_frame_of_original_file": self.startFrame(),
			"NumberOfFrames": self.numberOfFrames(),
			"Peak_value_of_10bits": 1020
		}
		if camera['Projection'] == 'Equirectangular':
			config.update({
				'Longitude_range_of_ERP': camera['Hor_range'][1] - camera['Hor_range'][0],
				'Latitude_range_of_ERP': camera['Ver_range'][1] - camera['Ver_range'][0]
			})
		return config

	def saveWspsnrJson(self):
		with open(self.wspsnrPath(), 'w') as stream:
			json.dump(self.wspsnrParameters(), stream, indent=4, sort_keys=True, separators=(',', ': '))
			stream.write('\n')

	def saveTmivJson(self):
		for v in self.allTargetCameraNames():
			self.outputCameraName = v
			TestConfiguration.saveTmivJson(self)
		if self.anchorId == 'A97' or self.anchorId == 'A17' or self.anchorId == 'V17' or self.anchorId == 'E97' or self.anchorId == 'E17':
			for v in self.allSourceCameraNames():
				self.outputCameraName = v
				self.saveWspsnrJson()

	def allTargetCameraNames(self):
		poseTraces = ['p01', 'p02', 'p03']
		if self.anchorId == 'R97' or self.anchorId == 'R17':
			return poseTraces
		return self.allSourceCameraNames() + poseTraces
	
class EncoderConfiguration(TestConfiguration):
	def __init__(self, sourceDir, anchorId, seqId):
		TestConfiguration.__init__(self, sourceDir, anchorId, seqId, 'R0')
		print("Encoder configuration: {} S{}".format(anchorId, seqId))

	def sourceCameraNames(self):
		if self.anchorId == 'V17':
			return {
				'A': [ 'v0', 'v7', 'v8', 'v9', 'v10', 'v11', 'v12', 'v13', 'v14' ],
				'B': [ 'v0', 'v1', 'v4', 'v8', 'v11', 'v12', 'v13', 'v17' ],
				'C': [ 'v1', 'v4', 'v5', 'v8', 'v9' ],
				'D': [ 'v0', 'v3', 'v5', 'v6', 'v9', 'v10', 'v12', 'v15' ],
				'E': [ 'v1', 'v3', 'v5', 'v7', 'v9', 'v11', 'v13' ],
				'J': [ 'v00', 'v02', 'v04', 'v10', 'v12', 'v14', 'v20', 'v22', 'v24' ],
				'L': [ 'v00', 'v02', 'v04', 'v06', 'v08' ],
				'N': [ 'v0', 'v1', 'v3', 'v5', 'v7', 'v9' ],
				'P': [ 'v0', 'v2', 'v4', 'v6', 'v8' ],
				'U': [ 'v0', 'v2', 'v4', 'v6', 'v8' ],
				'T': [ 'v0', 'v2', 'v4', 'v6', 'v8' ]
			}[self.seqId]
		return self.allSourceCameraNames()

	def numberOfCodedSourceViews(self):
		return len(self.sourceCameraNames())

	def maxEntities(self):
		if self.anchorId == 'E97' or self.anchorId == 'E17':
			return 25
		return 1

	def intraPeriod(self):
		return 32

	def firstSourceCameraName(self):
		return self.viewNameFormat().format(self.firstSourceView())

	def firstSourceCamera(self):
		for camera in self.sequenceParams['cameras']:
			if camera['Name'] == self.firstSourceCameraName():
				return camera
		raise 'Could not find first source camera in sequence configuration'

	def viewWidth(self):
		return self.firstSourceCamera()['Resolution'][0]

	def viewHeight(self):
		return self.firstSourceCamera()['Resolution'][1]


	def geometryDownscaleFactor(self):
		if self.anchorId == 'A97' or self.anchorId == 'A17':
			return 2
		return 1

	def atlasWidth(self):
		return checkAlignment(self.viewWidth())

	def atlasHeight(self):
		if self.anchorId == 'A97' or self.anchorId == 'A17' or self.anchorId == 'E97' or self.anchorId == 'E17':
			return checkAlignment({
				'A': 4096,
				'B': 2736,
				'C': 2048,
				'D': 2736,
				'E': 1456,
				'J': 1456,
				'L': 2912,
				'N': 2736,
				'P': 2912,
				'U': 2912,
				'T': 2912
			}[self.seqId])
		return checkAlignment(self.viewHeight())

	def geometryWidth(self):
		return checkAlignment(self.atlasWidth() // geometryDownscaleFactor())

	def geometryHeight(self):
		return checkAlignment(self.atlasHeight() // geometryDownscaleFactor())

	def viewOptimizerMethod(self):
		if self.anchorId == 'V17' or self.anchorId == 'R17' or self.anchorId == 'R97':
			return 'NoViewOptimizer'
		return 'ViewReducer'

	def sourceTexturePathFmt(self):
		return '{}_texture_{}x{}_yuv420p10le.yuv'

	def sourceDepthBitDepth(self):
		return self.firstSourceCamera()['BitDepthDepth']

	def sourceDepthVideoFormat(self):
		return {
			8: 'yuv420p',
			10: 'yuv420p10le',
			16: 'yuv420p16le'
		}[self.sourceDepthBitDepth()]
	
	def sourceDepthPathFmt(self):
		return '{{}}_depth_{{}}x{{}}_{}.yuv'.format(self.sourceDepthVideoFormat())
		
	def sourceEntityBitDepth(self):
		return 8

	def sourceEntityPathFmt(self):
		return '{}_entity_{}x{}_yuv420p.yuv'

	def omafV1CompatibleFlag(self):
		# Just to do something slightly more interesting than False
		return self.seqId == 'A' or self.seqId == 'C'
	
	def depthQualityAssessor(self):
		return {
			'blendingFactor': 0.03,
			'maxOutlierRatio': 0.001
		}

	def synthesizer(self):
		return {
			'rayAngleParameter': 10,
			'depthParameter': 50,
			'stretchingParameter': 3,
			'maxStretching': 5
		}
	
	def pruner(self):
		config = self.synthesizer()
		config.update({
			'maxDepthError': 0.1,
			'erode': 2,
			'dilate': 5
		})
		return config

	def packer(self):
		return {
			'Alignment': 8,
			'MinPatchSize': 16,
			'Overlap': 1,
			'PiP': 1
		}

	def lumaSamplesPerView(self):
		return 2 * self.atlasWidth() * self.atlasHeight()

	def maxLumaSamplesPerFrame(self):
		if self.anchorId == 'A97' or self.anchorId == 'A17':
			return {
				'A': 1,
				'B': 3,
				'C': 2,
				'D': 1,
				'E': 2,
				'J': 2,
				'L': 1,
				'N': 3,
				'P': 1,
				'U': 1,
				'T': 1
			}[self.seqId] * self.lumaSamplesPerView()
		if self.anchorId == 'E97' or self.anchorId == 'E17':
			return 4 * self.lumaSamplesPerView()
		return 0
		
	def atlasConstructorMethod(self):
		if self.anchorId == 'E97' or self.anchorId == 'E17':
			return 'EntityBasedAtlasConstructor'
		return 'AtlasConstructor'

	def atlasConstructor(self):
		config = {
			'PrunerMethod': 'HierarchicalPruner',
			'HierarchicalPruner': self.pruner(),
			'AggregatorMethod': 'Aggregator',
			'Aggregator': {},
			'PackerMethod': 'Packer',
			'Packer': self.packer(),
			'AtlasResolution': [
				self.atlasWidth(),
				self.atlasHeight()
			],
			'MaxLumaSamplesPerFrame': self.maxLumaSamplesPerFrame()
		}
		if self.anchorId == 'E97' or self.anchorId == 'E17':
			config['EntityEncodeRange'] = [0, self.maxEntities()]
		return config

	def depthOccupancy(self):
		return {
			'depthOccMapThresholdIfSet': 64
		}

	def encoder(self):
		return {
			'ViewOptimizerMethod': self.viewOptimizerMethod(),
			self.viewOptimizerMethod(): {},
			'AtlasConstructorMethod': self.atlasConstructorMethod(),
			self.atlasConstructorMethod(): self.atlasConstructor(),
			'DepthOccupancyMethod': 'DepthOccupancy',
			'DepthOccupancy': self.depthOccupancy(),
			'geometryScaleEnabledFlag': self.geometryDownscaleFactor() > 1
		}

	def numGroups(self):
		if self.anchorId == 'A97' or self.anchorId == 'A17' or self.anchorId == 'E97' or self.anchorId == 'E17':
			if self.firstSourceCamera()['Projection'] == 'Perspective':
				return 3
		return 1

	def parameters(self):
		config = TestConfiguration.parameters(self)
		config.update({
			'numGroups': self.numGroups(),
			'maxEntities': self.maxEntities(),
			'startFrame': self.startFrame(),
			'intraPeriod': self.intraPeriod(),
			'SourceCameraNames': self.sourceCameraNames(),
			'SourceTexturePathFmt': self.sourceTexturePathFmt(),
			'SourceDepthPathFmt': self.sourceDepthPathFmt(),
			'SourceDepthBitDepth': self.sourceDepthBitDepth(),
			'OmafV1CompatibleFlag': self.omafV1CompatibleFlag(),
			'EncoderMethod': 'GroupBasedEncoder',
			'GroupBasedEncoder': self.encoder(),
			'DepthQualityAssessorMethod': 'DepthQualityAssessor',
			'DepthQualityAssessor': self.depthQualityAssessor()
		})
		if self.anchorId == 'E17' or self.anchorId == 'E97':
		    config.update({
			    'SourceEntityPathFmt': self.sourceEntityPathFmt(),
			    'SourceEntityBitDepth': self.sourceEntityBitDepth()
		    })
		return config
		
	def path(self):
		return '{0}/S{1}/TMIV_{0}_S{1}.json'.format(self.anchorId, self.seqId)

	def saveHmCfg(self, component, scale):
		path = '{0}/S{2}/HM_{0}_{1}_S{2}.cfg'.format(self.anchorId, component, self.seqId)
		with open(path, 'w') as stream:
			stream.write('InputBitDepth: 10\n')
			stream.write('InputChromaFormat: 420\n')
			stream.write('FrameRate: 30\n')
			stream.write('FrameSkip: 0\n')
			stream.write('SourceWidth: {}\n'.format(self.atlasWidth()//scale))
			stream.write('SourceHeight: {}\n'.format(self.atlasHeight()//scale))
			stream.write('FramesToBeEncoded: {}\n'.format(self.numberOfFrames()))
			stream.write('SEIDecodedPictureHash: 1\n')
			stream.write('Level: 5.2\n')

def generate(anchorIds, seqIds, testPoints):
	for anchorId in anchorIds:
		for seqId in seqIds:
			config = EncoderConfiguration(sourceDir, anchorId, seqId)
			config.saveTmivJson()
			if len(testPoints) > 1:
				config.saveHmCfg('Tt', 1)
				config.saveHmCfg('Td', config.geometryDownscaleFactor())


	for testPoint in testPoints:
		for anchorId in anchorIds:
			for seqId in seqIds:
				config = DecoderConfiguration(sourceDir, anchorId, seqId, testPoint)
				config.saveTmivJson()

if __name__ == '__main__':
	if sys.version_info[0] < 3:
		print ('Error: Python version < 3')
		exit(-1)

	if len(sys.argv) > 2 or len(sys.argv) == 2 and sys.argv[1] == '--help':
		print('Usage: [python3] ./make_tmiv_configs.py [SOURCE_DIRECTORY]')
		print()
		print('The script without parameters is used to create the archive for inclusion with the CTC.')
		print()
		print('SOURCE_DIRECTORY may use replacement fields {0} for seqId (A) or {1} for seqName (ClassroomVideo).')
		print('Specifying SOURCE_DIRECTORY activates absolute paths to the configuration files.')
		print('The script should be run from the root of the output directory structure.')
		exit(1)

	sourceDir = '.'
	if len(sys.argv) == 2:
		sourceDir = sys.argv[1]

	allSeqIds = ['A', 'B', 'C', 'D', 'E', 'J', 'L', 'N','P', 'U', 'T']
	allTestPoints = ['R0', 'QP1', 'QP2', 'QP3', 'QP4', 'QP5']	

	generate(['R17', 'R97'], allSeqIds, ['R0'])
	generate(['A17', 'A97', 'V17'], allSeqIds, allTestPoints)
	generate(['E17', 'E97'], ['B'], allTestPoints)
