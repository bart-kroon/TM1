#!/usr/bin/env python3

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

import argparse
import json
from pathlib import Path


def scale_json_file(source_json_file: Path, scale_factor: float, output_json_file: Path) -> None:
    json_content = json.loads(source_json_file.read_text())
    for camera in json_content["cameras"]:
        scale_camera(camera, scale_factor)
    output_json_file.write_text(json.dumps(json_content, sort_keys=True, indent=4))


def scale_camera(camera, scale_factor):
    camera["Resolution"] = [int(element * scale_factor) for element in camera["Resolution"]]
    if camera["Projection"] == "Perspective":
        camera["Principle_point"] = [
            element * scale_factor for element in camera["Principle_point"]
        ]
        camera["Focal"] = [element * scale_factor for element in camera["Focal"]]


def parse_args():
    parser = argparse.ArgumentParser(description="Change the resolution of a sequence json")
    parser.add_argument("source_json_file", type=Path, help="Existing json file for reading")
    parser.add_argument(
        "scaling_factor",
        type=float,
        help="Scaling factor: output_resolution = input_resolution * scaling-factor",
    )
    parser.add_argument("output_json_file", type=Path, help="Output file with scaled entries")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    scale_json_file(args.source_json_file, args.scaling_factor, args.output_json_file)
