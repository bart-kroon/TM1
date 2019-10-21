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

# See Usage description at the end of the script
ctc_archive = False
source_directory = '.'

class DecoderConfiguration:
	def __init__(self, anchorId, seqId, testPoint):
		self.anchorId = anchorId
		self.seqId = seqId
		self.testPoint = testPoint
		with open(self.sequenceJsonPath(), 'r') as stream:			
			self.sequenceParams = json.load(stream)

	def numberOfFrames(self):
		if self.anchorId == 'A97':
			return 97
		return 17

	def intraPeriod(self):
		return 32

	def sourceDirectory(self):
		return source_directory.format(seqId, self.seqName())

	def seqName(self):
		return {
			'A': 'ClassroomVideo',
			'B': 'TechnicolorMuseum',
			'C': 'TechnicolorHijack',
			'D': 'TechnicolorPainter',
			'E': 'IntelFrog',
			'J': 'OrangeKitchen',
			'L': 'PoznanFencing'
		}[self.seqId]

	def sourceCameraParameters(self):
		if ctc_archive:
			return '{}.json'.format(self.seqName())	
		return self.sequenceJsonPath()

	def configPath(self):
		return os.path.dirname(sys.argv[0])

	def sequenceJsonPath(self):
		return os.path.realpath('{}/sequences/{}.json'.format(self.configPath(), self.seqName()))

	def firstSourceCamera(self):
		for camera in self.sequenceParams['cameras']:
			if camera['Name'] == self.firstSourceCameraName():
				return camera
		raise 'Could not find first source camera in sequence configuration'

	def viewWidth(self):
		return self.firstSourceCamera()['Resolution'][0]

	def viewHeight(self):
		return self.firstSourceCamera()['Resolution'][1]

	def atlasWidth(self):
		return self.viewWidth()

	def atlasHeight(self):
		if anchorId == 'A97' or anchorId == 'A17':
			return {
				'A': 3072,
				'B': 2368,
				'C': 5120,
				'D': 1088,
				'E': 1984,
				'J': 1080,
				'L': 1080
			}[self.seqId]
		return self.viewWidth()

	def viewportWidth(self):
		if self.outputCameraName()[0] == 'p':
			return {
				'A': 2048,
				'B': 2048,
				'C': 2048,
				'D': 1920,
				'E': 1920,
				'J': 1920,
				'L': 1920
			}[self.seqId]
		return self.viewWidth()

	def viewportHeight(self):
		if self.outputCameraName()[0] == 'p':
			return {
				'A': 2048,
				'B': 2048,
				'C': 2048,
				'D': 1080,
				'E': 1080,
				'J': 1080,
				'L': 1080
			}[self.seqId]
		return self.viewHeight()

	def atlasTexturePathFmt(self):
		return 'ATL_S{}_{}_Tt_c%02d_{}x{}_yuv420p10le.yuv'.format(self.seqId, self.testPoint, self.atlasWidth(), self.atlasHeight())

	def atlasDepthPathFmt(self):
		return 'ATL_S{}_{}_Td_c%02d_{}x{}_yuv420p10le.yuv'.format(self.seqId, self.testPoint, self.atlasWidth(), self.atlasHeight())

	def atlasMetadataPath(self):
		return 'ATL_S{}_{}_Tm_c00.bit'.format(self.seqId, self.testPoint)

	def outputDirectory(self):
		if ctc_archive:
			return '.'
		return os.path.realpath(os.path.dirname(self.path()))

	def outputTexturePath(self):
		return '{}_S{}_{}_Tt_{}_{}x{}_yuv420p10le.yuv'.format(
			self.anchorId, self.seqId, self.testPoint, self.outputCameraName(), self.viewportWidth(), self.viewportHeight())

	def outputCameraName(self):
		return self.viewNameFormat().format(1)

	def synthesizer(self):
		return {
			'rayAngleParameter': 10,
			'depthParameter': 50,
			'stretchingParameter': 3,
			'maxStretching': 5
		}

	def numberOfSourceViews(self):
		return {
			'A': 15,
			'B': 24,
			'C': 10,
			'D': 16,
			'E': 13,
			'J': 25,
			'L': 10
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

	def sourceCameraNames(self):
		if self.anchorId == 'V17':
			return {
				'A': [ 'v0', 'v7', 'v8', 'v9', 'v10', 'v11', 'v12', 'v13', 'v14' ],
				'B': [ 'v0', 'v1', 'v4', 'v8', 'v11', 'v12', 'v13', 'v17' ],
				'C': [ 'v1', 'v4', 'v5', 'v8', 'v9' ],
				'D': [ 'v0', 'v3', 'v5', 'v6', 'v9', 'v10', 'v12', 'v15' ],
				'E': [ 'v1', 'v3', 'v5', 'v7', 'v9', 'v11', 'v13' ],
				'J': [ 'v00', 'v02', 'v04', 'v10', 'v12', 'v14', 'v20', 'v22', 'v24' ],
				'L': [ 'v00', 'v02', 'v04', 'v06', 'v08' ]
			}[self.seqId]
		return self.allSourceCameraNames()

	def firstSourceCameraName(self):
		return self.viewNameFormat().format(self.firstSourceView())

	def numberOfCodedSourceViews(self):
		len(self.sourceCameraNames())

	def useMultipassRenderer(self):
		return self.anchorId == 'V17'

	def rendererMethod(self):
		if self.useMultipassRenderer():
			return 'MultipassRenderer'
		return 'Renderer'

	def renderer(self):
		config = {
			'SynthesizerMethod': 'Synthesizer',
			'Synthesizer': self.synthesizer(),
			'InpainterMethod': 'Inpainter',
			'Inpainter': {}
		}
		if self.useMultipassRenderer():
			config.update({
				'NumberOfPasses': 3,
				'NumberOfViewsPerPass': [2, 4, self.numberOfCodedSourceViews()]
			})
		return config

	def poseTraceBasename(self):
		return '{}{}.csv'.format(self.seqId, self.outputCameraName())

	def poseTracePath(self):
		if ctc_archive:
			return self.poseTraceBasename()
		return os.path.realpath('{}/pose_traces/{}'.format(self.configPath(), self.poseTraceBasename()))

	def parameters(self):
		config = {
			'numberOfFrames': self.numberOfFrames(),
			'intraPeriod': self.intraPeriod(),
			'SourceDirectory': self.sourceDirectory(),
			'SourceCameraParameters': self.sourceCameraParameters(),
			'AtlasTexturePathFmt': self.atlasTexturePathFmt(),
			'AtlasDepthPathFmt': self.atlasDepthPathFmt(),
			'AtlasMetadataPath': self.atlasMetadataPath(),
			'OutputDirectory': self.outputDirectory(),
			'OutputTexturePath': self.outputTexturePath(),
			'OutputCameraName': self.outputCameraName(),
			'DecoderMethod': 'Decoder',
			'Decoder': {
				'AtlasDeconstructorMethod': 'AtlasDeconstructor',
				'AtlasDeconstructor': {},
				'RendererMethod': self.rendererMethod(),
				self.rendererMethod(): self.renderer()
			}
		}
		if self.outputCameraName()[0] == 'p':
			config['OutputCameraName'] = 'viewport'
			config['PoseTracePath'] = self.poseTracePath()
			if self.anchorId == 'A97':
				config['extraNumberOfFrames'] = 300 - 97
		return config

	def testPointUnlessR0(self):
		if self.testPoint == 'R0':
			return '.'
		return self.testPoint

	def path(self):
		return '{0}/S{1}/{2}/{0}_S{1}_{3}_{4}.json'.format(
			self.anchorId, self.seqId, self.testPointUnlessR0(), self.testPoint, self.outputCameraName())

	def saveTmivJson(self):	
		print(self.path())
		os.makedirs(os.path.dirname(self.path()), exist_ok = True)
		with open(self.path(), 'w') as stream:
			json.dump(self.parameters(), stream, indent=4, sort_keys=True, separators=(',', ': '))
			stream.write('\n')


# Iterate over target views and pose traces
class AllDecoderConfigurations(DecoderConfiguration):
	def saveTmivJson(self):
		for v in self.allTargetCameraNames():
			self.overrideOutputCameraName = v
			DecoderConfiguration.saveTmivJson(self)

	def outputCameraName(self):
		return self.overrideOutputCameraName

	def allTargetCameraNames(self):
		return self.allSourceCameraNames() + ['p01', 'p02', 'p03']

class EncoderConfiguration(DecoderConfiguration):
	def __init__(self, anchorId, seqId):
		DecoderConfiguration.__init__(self, anchorId, seqId, 'R0')

	def startFrame(self):
		# TODO(BK): Decide on startFrame for 17-frame anchors
		return {
			'A': 23,
			'B': 100,
			'C': 0,
			'D': 10,
			'E': 135,
			'J': 0,
			'L': 30
		}[self.seqId]		

	def viewOptimizerMethod(self):
		if anchorId == 'V17':
			return 'NoViewOptimizer'
		return 'ViewReducer'

	def sourceTexturePathFmt(self):
		return '%s_texture_{}x{}_yuv420p10le.yuv'.format(self.viewWidth(), self.viewHeight())

	def sourceDepthBitDepth(self):
		return self.firstSourceCamera()['BitDepthDepth']

	def sourceDepthVideoFormat(self):
		return {
			8: 'yuv420p',
			10: 'yuv420p10le',
			16: 'yuv420p16le'
		}[self.sourceDepthBitDepth()]

	def sourceDepthPathFmt(self):
		return '%s_depth_{}x{}_{}.yuv'.format(self.viewWidth(), self.viewHeight(), self.sourceDepthVideoFormat())

	def omafV1CompatibleFlag(self):
		# Just to do something slightly more interesting than False
		return self.seqId == 'A' or self.seqId == 'C'

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
				'D': 4,
				'E': 3,
				'J': 4,
				'L': 3
			}[self.seqId] * self.lumaSamplesPerView()
		return 0

	def atlasConstructor(self):
		return {
			'PrunerMethod': 'Pruner',
			'Pruner': self.pruner(),
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

	def encoder(self):		
		return {
			'ViewOptimizerMethod': self.viewOptimizerMethod(),
			self.viewOptimizerMethod(): {},
			'AtlasConstructorMethod': 'AtlasConstructor',
			'AtlasConstructor': self.atlasConstructor()
		}

	def parameters(self):
		# The encoder configuration includes the decoder configuration for testing.
		# Enabling reconstruction will run the decoder while encoding.
		config = DecoderConfiguration.parameters(self)
		config['reconstruct'] = False

		config['startFrame'] = self.startFrame()
		config['SourceTexturePathFmt'] = self.sourceTexturePathFmt()
		config['SourceDepthPathFmt'] = self.sourceDepthPathFmt()
		config['SourceDepthBitDepth'] = self.sourceDepthBitDepth()
		config['SourceCameraNames'] = self.sourceCameraNames()
		config['OmafV1CompatibleFlag'] = self.omafV1CompatibleFlag()
		config['EncoderMethod'] = 'Encoder'
		config['Encoder'] = self.encoder()
		return config
		
	def path(self):
		return '{0}/S{1}/TMIV_{0}_S{1}.json'.format(self.anchorId, self.seqId)

	def saveHmCfg(self):
		path = '{0}/S{1}/HM_{0}_S{1}.cfg'.format(self.anchorId, self.seqId)
		print(path)
		with open(path, 'w') as stream:
			stream.write('InputBitDepth: 10\n')
			stream.write('InputChromaFormat: 420\n')
			stream.write('FrameRate: 30\n')
			stream.write('FrameSkip: 0\n')
			stream.write('SourceWidth: {}\n'.format(self.atlasWidth()))
			stream.write('SourceHeight: {}\n'.format(self.atlasHeight()))
			stream.write('FramesToBeEncoded: {}\n'.format(self.numberOfFrames()))
			stream.write('SEIDecodedPictureHash: 1\n')
			stream.write('Level: 5.2\n')

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

	if len(sys.argv) == 2:
		source_directory = sys.argv[1]
		ctc_archive = False

	anchors = [ 'A97', 'A17', 'V17' ]
	seqIds = ['A', 'B', 'C', 'D', 'E', 'J', 'L']
	testPoints = ['R0', 'QP1', 'QP2', 'QP3', 'QP4', 'QP5']

	for anchorId in anchors:
		for seqId in seqIds:
			config = EncoderConfiguration(anchorId, seqId)
			config.saveTmivJson()
			config.saveHmCfg()

	for testPoint in testPoints:
		for anchorId in anchors:
			for seqId in seqIds:
				config = AllDecoderConfigurations(anchorId, seqId, testPoint)
				config.saveTmivJson()
