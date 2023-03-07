# The copyright in this software is being made available under the BSD
# License, included below. This software may be subject to other third party
# and contributor rights, including patent rights, and no such rights are
# granted under this license.
#
# Copyright (c) 2010-2023, ISO/IEC
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
from pathlib import Path
import pytest
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
TOOLS_DIR = SCRIPT_DIR.parent
sys.path.append(str(TOOLS_DIR))

from scale_sequence_json import scale_json_file

PERSPECTIVE_SOURCE_FILE_CONTENT = """{
    "Version": "4.0",
    "Content_name": "Frog",
    "lengthsInMeters": true,
    "cameras": [
        {
            "ColorSpace": "YUV420",
            "Depth_range": [
                0.3,
                1.62
            ],
            "Focal": [
                1856.80,
                1856.80
            ],
            "Name": "viewport",
            "Principle_point": [
                960,
                540
            ],
            "Projection": "Perspective",
            "Resolution": [
                1920,
                1080
            ]
        },
        {
            "ColorSpace": "YUV420",
            "DepthColorSpace": "YUV420",
            "Focal": [
                1546.74,
                1547.76
            ],
            "Name": "v1",
            "Position": [
                0.000348132,
                0.217795,
                -0.000293372
            ],
            "Principle_point": [
                980.168,
                534.722
            ],
            "Projection": "Perspective",
            "Resolution": [
                4000,
                2000
            ]
        },
        {
            "Name": "ERP cam",
            "ColorSpace": "YUV444",
            "Position": [
                0.000348132,
                0.217795,
                -0.000293372
            ],
            "Projection": "Equirectangular",
            "Resolution": [
                1280,
                720
            ]
        }
    ]
}
"""


@pytest.mark.parametrize(
    "scale_factor, expected_entries",
    [
        (
            0.5,
            {
                "viewport": {
                    "Resolution": [960, 540],
                    "Principle_point": [480, 270],
                    "Focal": [928.4, 928.4],
                },
                "v1": {
                    "Resolution": [2000, 1000],
                    "Principle_point": [490.084, 267.361],
                    "Focal": [773.37, 773.88],
                },
                "ERP cam": {
                    "Resolution": [640, 360],
                },
            },
        ),
        (
            0.1,
            {
                "viewport": {
                    "Resolution": [192, 108],
                    "Principle_point": [96.0, 54.0],
                    "Focal": [185.68, 185.68],
                },
                "v1": {
                    "Resolution": [400, 200],
                    "Principle_point": [98.0168, 53.4722],
                    "Focal": [154.674, 154.776],
                },
                "ERP cam": {
                    "Resolution": [128, 72],
                },
            },
        ),
    ],
)
def test_scale_file(fs, scale_factor, expected_entries):
    source_file = Path("input.json")

    fs.create_file(source_file, contents=PERSPECTIVE_SOURCE_FILE_CONTENT)
    destination_file = Path("output.json")

    scale_json_file(source_file, scale_factor, destination_file)

    # Check changed entries
    changed_config = json.loads(destination_file.read_text())
    for camera in changed_config["cameras"]:
        assert camera["Resolution"] == expected_entries[camera["Name"]]["Resolution"]
        if camera["Projection"] == "Perspective":
            assert camera["Principle_point"] == expected_entries[camera["Name"]]["Principle_point"]
            assert camera["Focal"] == expected_entries[camera["Name"]]["Focal"]

    # Check some of the unchanged entries
    original_config = json.loads(source_file.read_text())
    for entry in ("Content_name", "Version"):
        assert original_config[entry] == changed_config[entry]

    for changed_camera, original_camera in zip(
        changed_config["cameras"], original_config["cameras"]
    ):
        for entry in ("Projection", "Name", "ColorSpace"):
            assert original_camera[entry] == changed_camera[entry]
