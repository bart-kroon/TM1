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
import multiprocessing
import pathlib
import subprocess

SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
REPO_DIR = SCRIPT_DIR.parents[1]


def check_for_empty_line_after_curly_brace(text):
    return text.replace("{\n\n", "{\n")


def apply_all_checks_to_a_single_cpp_file(file):
    with open(file, mode="r") as stream:
        text = stream.read()

    original = text
    text = check_for_empty_line_after_curly_brace(text)

    if original != text:
        print("At least one check triggered on", file)
        with open(file, mode="w") as stream:
            stream.write(text)


def get_list_of_changed_files(target_branch):
    return map(
        lambda x: pathlib.Path(x),
        subprocess.run(
            [
                "git",
                "diff",
                "--no-merges",
                "--name-only",
                "--diff-filter=d",
                target_branch,
            ],
            cwd=REPO_DIR,
            check=True,
            stdout=subprocess.STDOUT,
        ).stdout.splitlines(),
    )


def get_list_of_files():
    return map(
        lambda x: pathlib.Path(x),
        subprocess.run(
            ["git", "ls-files"], cwd=REPO_DIR, check=True, stdout=subprocess.PIPE, text=True
        ).stdout.splitlines(),
    )


def filter_cpp_files(files):
    return list(filter(lambda f: f.suffix in (".cpp", ".hpp", ".h"), files))


def apply_all_checks_to_multiple_cpp_files(cpp_files):
    with multiprocessing.Pool(multiprocessing.cpu_count()) as pool:
        pool.map(apply_all_checks_to_a_single_cpp_file, cpp_files)


def parse_args():
    parser = argparse.ArgumentParser(description="Apply multiple project-specific checks")
    parser.add_argument(
        "-c", "--changed-only", help="Check only changed files", action="store_true"
    )
    parser.add_argument("file", help="Check specified file", nargs="*", type=pathlib.Path)
    parser.add_argument(
        "-b",
        "--target-branch",
        help="Target branch for changed files computation",
        default="origin/main",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    files = args.file

    if len(files) == 0:
        if args.changed_only:
            files = get_list_of_changed_files(args.target_branch)
        else:
            files = get_list_of_files()

    cpp_files = filter_cpp_files(files)
    apply_all_checks_to_multiple_cpp_files(cpp_files)
    exit(0)
