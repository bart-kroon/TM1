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


import json
import sys
from pathlib import Path
from subprocess import CompletedProcess, run
from concurrent.futures import ThreadPoolExecutor, as_completed


CLANG_TIDY = "clang-tidy-14"

error_count = 0


def changedCppFiles():
    return filter(
        lambda path: path.suffix == ".cpp",
        map(
            lambda line: Path(line).resolve(),
            run(
                ["git", "diff", "--name-only", "--diff-filter=ACMRTUXB", "origin/main"],
                encoding="utf8",
                capture_output=True,
                check=True,
            ).stdout.split(),
        ),
    )


class FileNotInBuild(Exception):
    def __init__(self, file: str):
        self.file = file


def compileCommand(file: Path, compile_commands: dict):
    for item in compile_commands:
        if item["file"] == str(file):
            return item["directory"], item["command"]

    raise FileNotInBuild(file)


def runClangTidy(file: Path, compile_commands: dict):
    global error_count

    print_path = file.relative_to(Path.cwd())

    if error_count > 0:
        return print_path

    try:
        directory, command = compileCommand(file, compile_commands)

        cmd = [CLANG_TIDY, "--use-color", "--quiet"]

        if str(file).endswith(".test.cpp"):
            cmd.append(
                "-checks=-readability-function-size,"
                "-readability-magic-numbers,"
                "-readability-function-cognitive-complexity"
            )

        cmd += [
            "--extra-arg-before=--driver-mode=g++",
            file,
            "--",
        ] + command.split()

        result = run(cmd, cwd=directory, capture_output=True)
        result.file = print_path
        return result
    except FileNotInBuild as e:
        return e


def main():
    global error_count

    with open("build/compile_commands.json", encoding="utf8") as stream:
        compile_commands = json.load(stream)

    with ThreadPoolExecutor(max_workers=6) as executor:
        files = changedCppFiles()
        futures = [executor.submit(runClangTidy, file, compile_commands) for file in files]

    for future in as_completed(futures):
        try:
            result = future.result()

            if isinstance(result, Path):
                print(f"Skipped {result} due to earlier errors.")
            elif isinstance(result, FileNotInBuild):
                print(f"Skipped {result.file} because the file is not included in this build.")
            elif result.returncode == 0:
                print(f"{CLANG_TIDY}: {result.file}", flush=True)
            else:
                error_count += 1
                what = result.stdout.decode().replace(
                    "/builds/software/MPEG/Video/MIV/Software/TMIV/", ""
                )
                print(f"{CLANG_TIDY}: {result.file}:\n{what}", flush=True)
        except Exception as e:
            error_count += 1
            print(f"ERROR: {e}")


if __name__ == "__main__":
    main()
    sys.exit(error_count)
