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
import subprocess

CONFORMANCE_BITSTREAM_IDS = [
    "CB01",
    "CB02",
    "CB03",
    "CB04",
    "CB05.1",
    "CB05.2",
    "CB05.3",
    "CB06",
    "CB07.1",
    "CB07.2",
    "CB07.3",
    "CB08",
    "CB09",
    "CB10",
    "CB11",
    "CB12",
    "CB14",
    "CB15",
    "CB16",
    "CB17",
    "CB18",
    "CB19",
    "CB20",
]


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-b",
        "--bitstreams-dir",
        type=Path,
        required=True,
        help="Directory with unpacked MIV conformance bitstreams",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        required=True,
        help="Directory to which to write the log files",
    )
    parser.add_argument(
        "-t", "--tmiv-dir", type=Path, required=True, help="TMIV installation prefix"
    )
    parser.add_argument(
        "-i", "--conformance-bitstream-ids", type=str, nargs="*", default=CONFORMANCE_BITSTREAM_IDS
    )
    return parser.parse_args()


def main(
    bitstreams_dir: Path, output_dir: Path, tmiv_dir: Path, conformance_bitstream_ids: list[str]
):
    for id in conformance_bitstream_ids:
        id_s = id.replace(".", "_")

        test_decoder_conformance(
            tmiv_dir=tmiv_dir,
            bitstream_file=bitstreams_dir / id_s / f"{id_s}.bit",
            output_file=output_dir / f"{id_s}_test.dec",
            reference_file=bitstreams_dir / id_s / f"{id_s}.dec",
        )
        print(f"Conformance bitstream {id}: OK", flush=True)


def test_decoder_conformance(
    tmiv_dir: Path, bitstream_file: Path, output_file: Path, reference_file: Path
):
    run_tmiv_decoder_log(tmiv_dir, bitstream_file, output_file)
    check_conformance_log(output_file, reference_file)


def run_tmiv_decoder_log(tmiv_dir: Path, bitstream_file: Path, output_file: Path):
    executable = tmiv_dir / "bin" / "TmivDecoderLog"
    args = [executable, "-b", bitstream_file, "-o", output_file]

    output_file.parent.mkdir(parents=True, exist_ok=True)

    result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, encoding="utf8")

    if result.returncode != 0:
        print(f"> {' '.join(map(str, args))}")
        print(result.stdout.strip())
        raise RuntimeError(f"TmivDecoderLog exited with return code {result.returncode}")


def check_conformance_log(output_file: Path, reference_file: Path):
    with open(output_file, encoding="utf8") as stream:
        output = stream.readlines()

    with open(reference_file, encoding="utf8") as stream:
        reference = stream.readlines()

    if output != reference:
        raise RuntimeError(
            f"The actual decoder output log {output_file} does not match "
            f"with the reference decoder output log {reference_file}."
        )


if __name__ == "__main__":
    try:
        args = parse_arguments()
        main(**vars(args))
    except (RuntimeError, OSError, subprocess.CalledProcessError) as e:
        print(e)
        exit(1)
