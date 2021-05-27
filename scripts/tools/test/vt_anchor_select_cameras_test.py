# The copyright in this software is being made available under the BSD
# License, included below. This software may be subject to other third party
# and contributor rights, including patent rights, and no such rights are
# granted under this license.
#
# Copyright (c) 2010-2021, ISO/IEC
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

import pathlib
import pytest
import subprocess
import sys

scriptsToolsDir = pathlib.Path(__file__).parent.parent
tmivRootDir = scriptsToolsDir.parent.parent
scriptAtTestPath = scriptsToolsDir / "vt_anchor_select_cameras.py"
configCtcSequenceDir = tmivRootDir / "config" / "ctc" / "sequences"

sys.path.append(str(scriptsToolsDir))
print(scriptsToolsDir)
import vt_anchor_select_cameras


@pytest.mark.parametrize(
    "fileStem, expectedOutput",
    [
        ("A", "v3, v11\n"),
        ("B", "v3, v9, v15, v21\n"),
        ("C", "v2, v7\n"),
        ("D", "v1, v3, v5, v7, v9, v11, v13, v15\n"),
        ("E", "v1, v3, v5, v6, v8, v9, v11, v13\n"),
        ("I", "v0, v2, v4, v6, v8, v10, v12, v14\n"),
        ("J", "v01, v04, v07, v10, v14, v17, v20, v23\n"),
        ("L", "v0, v1, v2, v3, v4, v5, v6, v7, v8, v9\n"),
        ("M", "mpi\n"),
        ("N", "v1, v3, v6, v8\n"),
        ("O", "v00, v02, v04, v06, v08, v10, v12, v14\n"),
        ("P", "v0, v1, v2, v3, v4, v5, v6, v7, v8\n"),
        ("Q", "v1, v3, v6, v8\n"),
        ("R", "v01, v03, v06, v09, v11, v14, v17, v19\n"),
        ("T", "v0, v1, v2, v3, v4, v5, v6, v7, v8\n"),
        ("U", "v0, v1, v2, v3, v4, v5, v6, v7, v8\n"),
        ("V", "v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15\n"),
    ],
)
def test_integration_vt_anchor_select_cameras_default_params(fileStem, expectedOutput):
    configPath = configCtcSequenceDir / "{}.json".format(fileStem)

    result = subprocess.run(
        [sys.executable, scriptAtTestPath, configPath],
        stdout=subprocess.PIPE,
        text=True,
        check=True,
    )

    assert result.stdout == expectedOutput
    assert result.stderr is None


class SomeArgs:
    def __init__(self, fileStem):
        self.config = self.makeConfig(fileStem)
        self.leave_n_out = 2
        self.component_count = 1
        self.max_luma_sample_rate = 3e8

    def makeConfig(self, fileStem):
        cameraCount = {"A": 15, "D": 16, "T": 9}[fileStem]
        resolution = {"A": [4096, 2048], "D": [2048, 1088], "T": [1920, 1080]}[fileStem]
        frameRate = {"A": 30, "D": 30.0, "T": 25}[fileStem]

        return {
            "Fps": frameRate,
            "sourceCameraNames": list(map(lambda x: "v{}".format(x), range(cameraCount))),
            "cameras": list(
                map(
                    lambda x: {"Name": "v{}".format(x), "Resolution": resolution},
                    range(cameraCount),
                )
            ),
        }


@pytest.mark.parametrize(
    "fileStem, expectedOutput",
    [
        ("A", ["v7"]),
        ("D", ["v2", "v6", "v10", "v14"]),
        ("T", ["v0", "v2", "v4", "v6", "v8"]),
    ],
)
def test_VtAnchorSelectCameras_targetCameraNames(fileStem, expectedOutput):
    unit = vt_anchor_select_cameras.VtAnchorSelectCameras(SomeArgs(fileStem))

    assert unit.targetCameraNames() == expectedOutput


class HetrogeneousArgs:
    def __init__(self):
        self.config = {
            "Fps": 1,
            "cameras": [
                {"Name": "First camera", "Resolution": [1, 2]},
                {"Name": "Second camera", "Resolution": [3, 4]},
            ],
            "sourceCameraNames": ["First camera", "Second camera"],
        }
        self.leave_n_out = 0
        self.component_count = 1
        self.max_luma_sample_rate = 1.0


def test_VtAnchorSelectCameras_hetrogeneousResolution():
    unit = vt_anchor_select_cameras.VtAnchorSelectCameras(HetrogeneousArgs())

    with pytest.raises(RuntimeError) as e:
        unit.targetCameraNames()
