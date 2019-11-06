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
from make_tmiv_configs import *

class QpTuningEncoderConfiguration(EncoderConfiguration):
	def __init__(self, sourceDir, seqId):
		EncoderConfiguration.__init__(self, sourceDir, 'A17', seqId)

class AllQpTuningDecoderConfigurations(AllDecoderConfigurations):
	def __init__(self, sourceDir, seqId, textureQPs, deltaQPs):
		AllDecoderConfigurations.__init__(self, sourceDir, 'A17', seqId, 'R0')
		self.textureQPs = textureQPs
		self.deltaQPs = deltaQPs

	def saveTmivJson(self):
		for i in range(0, len(self.textureQPs)):
			self.textureQP = self.textureQPs[i]
			for j in range(0, len(self.deltaQPs)):
				deltaQP = self.deltaQPs[j]
				self.depthQP = self.textureQP + deltaQP
				self.testPoint = 'T{:02}D{:02}'.format(self.textureQP, self.depthQP)
				AllDecoderConfigurations.saveTmivJson(self)

	def atlasTexturePathFmt(self):
		return os.path.join('..',
			'ATL_S{}_Q{:02}_Tt_c%02d_{}x{}_yuv420p10le.yuv'.format(self.seqId, self.textureQP, self.atlasWidth(), self.atlasHeight()))

	def atlasDepthPathFmt(self):
		return os.path.join('..',
			'ATL_S{}_Q{:02}_Td_c%02d_{}x{}_yuv420p10le.yuv'.format(self.seqId, self.depthQP, self.atlasWidth(), self.atlasHeight()))

	def atlasMetadataPath(self):
		return os.path.join('..',
			'ATL_S{}_R0_Tm_c00.bit'.format(self.seqId))

if __name__ == '__main__':
	if sys.version_info[0] < 3:
		print ('Error: Python version < 3')
		exit(-1)

	if len(sys.argv) != 2 or len(sys.argv) == 2 and sys.argv[1] == '--help':
		print('Usage: [python3] ./qp_tuning_configs.py SOURCE_DIRECTORY')
		print()
		print('SOURCE_DIRECTORY may use replacement fields {0} for seqId (A) or {1} for seqName (ClassroomVideo).')
		print('The script should be run from the root of the output directory structure.')
		exit(1)

	sourceDir = sys.argv[1]

	for seqId in ['A', 'B', 'C', 'D', 'E', 'J', 'L', 'N']:
		config = QpTuningEncoderConfiguration(sourceDir, seqId)
		config.saveTmivJson()
		config.saveHmCfg()
		config = AllQpTuningDecoderConfigurations(sourceDir, seqId, [22, 27, 32, 37, 42], [-14, -10, -6, -3, 0])
		config.saveTmivJson()