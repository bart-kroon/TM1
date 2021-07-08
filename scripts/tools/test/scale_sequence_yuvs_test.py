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

from pathlib import Path
import pytest
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
TOOLS_DIR = SCRIPT_DIR.parent
sys.path.append(str(TOOLS_DIR))

from scale_sequence_yuvs import (
    create_ffmpeg_scaling_command,
    get_pixel_format,
    get_resolution,
    Resolution,
)


@pytest.mark.parametrize(
    "file_stem, expected_pixel_format",
    [
        ("T_Q1_p02_2048x2048_yuv420p10le", "yuv420p10le"),
        ("T_p02_2048x2048_yuv444p10le", "yuv444p10le"),
        ("T_2048x2048_yuv422p16le", "yuv422p16le"),
    ],
)
def test_get_pixel_format(file_stem, expected_pixel_format):
    assert get_pixel_format(file_stem) == expected_pixel_format


@pytest.mark.parametrize("file_stem", ["T_Q1_p02_2048x2048_yuv420p15le"])
def test_get_pixel_format_failure(file_stem):
    with pytest.raises(ValueError):
        get_pixel_format(file_stem)


def test_get_resolution():
    assert get_resolution("T_Q1_p02_2048x2048_yuv420p10le") == Resolution(2048, 2048)
    assert get_resolution("T_Q1_p02_1920x1080") == Resolution(1920, 1080)
    assert get_resolution("1024x768_yuv420p") == Resolution(1024, 768)
    assert get_resolution("8x4_yuv420p") == Resolution(8, 4)


@pytest.mark.parametrize(
    "file_stem",
    [
        "file_1920x_yuv",
        "file_yuv",
        "file_x12_yuv",
        "file_12x10_24x16_yuv",
    ],
)
def test_get_resolution_failure(file_stem):
    with pytest.raises(ValueError):
        get_resolution(file_stem)


def test_create_ffmpeg_scaling_command():
    yuv_file = Path("home") / "some" / "folder" / "T_Q1_p02_2048x2048_yuv420p10le.yuv"
    output_folder = Path("home") / "output"
    assert (
        create_ffmpeg_scaling_command(yuv_file, 0.5, output_folder)
        == "ffmpeg -s:v 2048x2048 -r 30 -pix_fmt yuv420p10le"
        + " -i home/some/folder/T_Q1_p02_2048x2048_yuv420p10le.yuv"
        + " -frames:v 3 -pix_fmt yuv420p10le -c:v rawvideo"
        + " -vf scale=1024:1024 home/output/T_Q1_p02_1024x1024_yuv420p10le.yuv"
    )


def test_create_ffmpeg_scaling_command_failure():
    with pytest.raises(ValueError):
        create_ffmpeg_scaling_command(Path("file.mp4"), 1, Path("/home"))
