#!/usr/bin/env python3

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

import argparse
from pathlib import Path
import re
from subprocess import run
import sys

SCRIPTS_DIR = Path(__file__).resolve().parent.parent
sys.path.append(str(SCRIPTS_DIR))

from common.video_types import Resolution


def create_commands_for_given_files(args):
    return [
        create_ffmpeg_scaling_command(f, args.scaling_factor, args.output_folder)
        for f in Path(args.sequence_folder).glob("*.yuv")
    ]


def create_ffmpeg_scaling_command(
    input_yuv_file: Path, scaling_factor: float, output_folder: Path
) -> str:
    if input_yuv_file.suffix.lower() != ".yuv":
        raise ValueError("Input must be a YUV file")

    input_resolution = get_resolution(input_yuv_file.stem)
    pixel_format = get_pixel_format(input_yuv_file.stem)
    output_resolution = input_resolution * scaling_factor
    output_yuv_file = output_folder / (
        input_yuv_file.stem.replace(str(input_resolution), str(output_resolution)) + ".yuv"
    )

    return (
        f"ffmpeg -s:v {input_resolution} -r 30 -pix_fmt {pixel_format} -i {input_yuv_file}"
        + f" -frames:v 3 -pix_fmt {pixel_format} -c:v rawvideo"
        + f" -vf scale={output_resolution.width}:{output_resolution.height} {output_yuv_file}"
    )


def get_pixel_format(file_stem: str):
    pattern = re.compile(r"yuv\d{3}p(1[0,6]le)?$")
    matches = pattern.search(file_stem)
    if matches is None:
        raise ValueError(f"Couldn't find any pixel format in file name {file_stem}")
    return matches.group(0)


def get_resolution(file_stem: str) -> Resolution:
    matches = re.findall(r"(\d+)x(\d+)", file_stem)
    if len(matches) != 1:
        raise ValueError(f"Couldn't find a unique resolution in file name {file_stem}")
    return Resolution(width=int(matches[0][0]), height=int(matches[0][1]))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Change the resolution of yuv files in a sequence folder using ffmpeg"
    )
    parser.add_argument(
        "sequence_folder",
        type=Path,
        help="Sequence directory containing YUV files",
    )
    parser.add_argument(
        "scaling_factor",
        type=float,
        help="Scaling factor: output_resolution = input_resolution * scaling-factor",
    )
    parser.add_argument(
        "output_folder",
        type=Path,
        help="Output directory for the new sequence",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if not args.output_folder.is_dir():
        args.output_folder.mkdir(parents=True)

    for command in create_commands_for_given_files(args):
        print(command)
        run(command, shell=True)
