#!/usr/bin/env python3

# The copyright in this software is being made available under the BSD
# License, included below. This software may be subject to other third party
# and contributor rights, including patent rights, and no such rights are
# granted under this license.
#
# Copyright (c) 2024, ISO/IEC
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
import sys

from install import load_presets, resolve_preset, build, test, install, run


def main(
    preset_names: list[str],
    skip_build: bool,
    skip_unit_tests: bool,
    skip_install: bool,
    skip_integration_tests: bool,
    skip_conformance_tests: bool,
    input_dir: Path,
    output_dir: Path,
    project_dir: Path,
    thread_count: int,
):
    if not skip_integration_tests or not skip_conformance_tests:
        check_that_test_data_exists(input_dir)

    presets = load_presets(project_dir)

    for preset_name in preset_names:
        preset = resolve_preset(preset_name, presets, project_dir)

        build_dir = preset["binaryDir"]

        if not skip_build:
            build(build_dir, thread_count)

        if not skip_unit_tests:
            test(build_dir)

        if not skip_install:
            install(build_dir)

        if not skip_integration_tests:
            integration_test(preset, input_dir, output_dir, project_dir, thread_count)

        if not skip_conformance_tests:
            decoder_conformance_test(preset, input_dir, output_dir, project_dir)
            vpcc_conformance_parse_test(preset, input_dir, output_dir, project_dir)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="Build and install TMIV, and run all available integration tests.",
        epilog="Make sure to run this script before marking a merge request as ready. "
        "Run this script from a suitable Python virtual environment, "
        "and also make sure that the environment is set-up for the intended compiler toolchain. "
        "For Microsoft Visual Studio, run in a x64 Native Tools Command Prompt.",
    )

    parser.add_argument(
        "preset_names",
        type=str,
        nargs="+",
        help="List of CMake configure presets for which to prebuild, build, test and install TMIV. "
        "Specifying none will still download dependencies. "
        "Use cmake --list-presets for a list of available presets.",
        default=[],
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Skip building of TMIV",
    )
    parser.add_argument(
        "--skip-unit-tests",
        action="store_true",
        help="Skip the unit/developer tests",
    )
    parser.add_argument(
        "--skip-install",
        action="store_true",
        help="Skip installation of TMIV",
    )
    parser.add_argument(
        "--skip-integration-tests",
        action="store_true",
        help="Skip the integration tests",
    )
    parser.add_argument(
        "--skip-conformance-tests",
        action="store_true",
        help="Skip conformance-related tests on MIV and V-PCC bitstreams",
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        help="Directory with test content, to be manually downloaded from the MPEG content server (see README.md, relative to --project-dir)",
        default=Path("in") / "test",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Output directory for all integration tests. "
        "A sub-directory will be made for each tested CMake preset (relative to --project-dir)",
        default=Path("out") / "test",
    )
    parser.add_argument(
        "--project-dir",
        type=Path,
        help="TMIV source directory (the directory with the README.md)",
        default=Path(__file__).parent.parent.resolve(),
    )
    parser.add_argument(
        "-j",
        "--thread-count",
        type=int,
        help="limit the number of parallel processes when building software or running tests",
        default=0,
    )

    args = parser.parse_args()

    args.input_dir = args.project_dir / args.input_dir
    args.output_dir = args.project_dir / args.output_dir

    return args


def check_that_test_data_exists(input_dir: Path):
    for sub_dir in ["B", "C", "conformance", "D", "E", "M", "N", "O", "vpcc_conformance"]:
        if not (input_dir / sub_dir).exists():
            raise RuntimeError(
                f"Missing test data in {input_dir}. Please log in on the MPEG content server, "
                "navigate to MPEG-I/Part12-ImmersiveVideo/tmiv_integration_test, "
                "download the zip-file, and unpack in that directory."
            )


def integration_test(
    preset: dict, input_dir: Path, output_dir: Path, project_dir: Path, thread_count: int
):
    args = [sys.executable, project_dir / "scripts" / "test" / "integration_test.py"]
    args += [preset["installDir"], project_dir, input_dir, output_dir / preset["name"]]
    args += ["-g", "git"]
    args += ["--md5sums-file", project_dir / "test" / f"{preset['name']}.md5"]

    if thread_count:
        args += ["-j", thread_count]

    run(args)


def decoder_conformance_test(preset: dict, input_dir: Path, output_dir: Path, project_dir: Path):
    args = [sys.executable, project_dir / "scripts" / "test" / "decoder_conformance_test.py"]
    args += ["-t", preset["installDir"]]
    args += ["-b", input_dir / "conformance"]
    args += ["-o", output_dir / preset["name"]]

    run(args)


def vpcc_conformance_parse_test(preset: dict, input_dir: Path, output_dir: Path, project_dir: Path):
    args = [sys.executable, project_dir / "scripts" / "test" / "vpcc_conformance_parse_test.py"]
    args += ["-t", preset["installDir"]]
    args += ["-b", input_dir / "vpcc_conformance"]
    args += ["-o", output_dir / preset["name"]]

    run(args)


if __name__ == "__main__":
    try:
        args = parse_arguments()
        main(**vars(args))
    except (RuntimeError, OSError, subprocess.CalledProcessError) as e:
        print(e)
        exit(1)
