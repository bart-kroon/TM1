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
import multiprocessing
import pathlib
import subprocess
import re

SCRIPT_DIR = pathlib.Path(__file__).resolve().parent
REPO_DIR = SCRIPT_DIR.parents[1]


class CodeQualityRules:
    def __init__(self, verbose, dry_run):
        self.verbose = verbose
        self.dry_run = dry_run

    ### code quality rules ###

    def remove_empty_lines_after_curly_brace(self, text):
        return text.replace("{\n\n", "{\n")

    def remove_iostream_header(self, text):
        return text.replace("#include <iostream>\n", "")

    def replace_platform_dependent_primitive_types(self, text):
        return re.sub(
            r"(//[^\n]+|main)?[^a-zA-Z0-9_\"]((un)?signed\s+)?(short|int|long|long long)[^a-zA-Z0-9_\"]",
            self.replace_platform_dependent_types_replace,
            text,
        )

    def replace_platform_dependent_types_replace(self, match):
        if self.verbose:
            print("Check: Replace platform dependent types with cstdint type aliases")
            for i in range(len(match.groups())):
                print("{}match[{}] = '{}'".format("" if i == 0 else "  ", i, match[i]))

        text = match[0]

        if match[1]:
            return text

        # Carve a hole for postfix operator ++/--
        if text == "(int)":
            return text

        replacement_type = {
            "short": "int16_t",
            "signed short": "int16_t",
            "unsigned short": "uint16_t",
            "int": "int32_t",
            "signed": "int32_t",
            "signed int": "int32_t",
            "long": "int32_t",
            "signed long": "int32_t",
            "unsigned": "uint32_t",
            "unsigned int": "uint32_t",
            "long long": "int64_t",
            "signed long long": "int64_t",
            "unsigned long": "uint32_t",
            "unsigned long long": "uint64_t",
        }[text[1:-1]]
        return "".join((text[0], replacement_type, text[-1]))

    def remove_std_primitive_types(self, text):
        return re.sub(
            r"std::(u?int(_fast|_least)?(8|16|32|64)_t|u?intmax_t|u?intptr_t|size_t|ptrdiff_t)[^a-zA-Z0-9_]",
            self.remove_std_primitive_types_replace,
            text,
        )

    def remove_std_primitive_types_replace(self, match):
        if self.verbose:
            print(
                "Check: Drop the std:: in front of C11 fundamental type typedefs, e.g. int16_t, size_t"
            )
            for i in range(len(match.groups())):
                print("{}match[{}] = '{}'".format("" if i == 0 else "  ", i, match[i]))

        return match[0][5:]

    def replace_function_style_cast_to_primitive_types_by_static_cast(self, text):
        return re.sub(
            r"(//[^\n]+)?(operator|function)?([^a-z0-9_:])(nullptr_t|signed|unsigned|short|long|int|bool|char|float|double|u?int(_fast|_least)?(8|16|32|64)_t|u?intmax_t|u?intptr_t|size_t|ptrdiff_t|streamoff)\(",
            self.replace_function_style_cast_to_primitive_types_by_static_cast_replace,
            text,
        )

    def replace_function_style_cast_to_primitive_types_by_static_cast_replace(self, match):
        if self.verbose:
            print("Check: Replace a function-style cast to primitive type with a static_cast")
            for i in range(len(match.groups())):
                print("{}match[{}] = '{}'".format("" if i == 0 else "  ", i, match[i]))

        if match[1] or match[2]:
            return match[0]
        return "{0}static_cast<{1}>(".format(match[3], match[4])

    def detect_c_style_cast(self, text):
        return re.sub(
            r"(//[^\n]+)?\((bool|char|float|double|u?int(_fast|_least)?(8|16|32|64)_t|u?intmax_t|u?intptr_t|size_t|ptrdiff_t|streamoff)\)[^\)>]",
            self.detect_c_style_cast_replace,
            text,
        )

    def detect_c_style_cast_replace(self, match):
        if self.verbose:
            print("Check: Replace a c-style cast to primitive type with a static_cast")
            for i in range(len(match.groups())):
                print("{}match[{}] = '{}'".format("" if i == 0 else "  ", i, match[i]))

        if match[1]:
            return match[0]

        type_ = match[0][1:-1]
        raise RuntimeError(
            f"Replace c-style cast {match[0]} with Common::downCast<{type_}>( ) or static_cast<{type_}>( )"
        )

    def detect_fmt_print(self, text):
        return re.sub(r"fmt::print\(\"", self.detect_fmt_print_replace, text)

    def detect_fmt_print_replace(self, match):
        raise RuntimeError(
            "Replace `fmt::print(fmt, args...)` with `Common::logInfo(fmt, args...)`. "
            "Note that `fmt::print(stream, fmt, args...)` with `std::ostringstream &stream` is allowed."
        )

    ### other logic ###

    def apply_all_rules_to_a_single_cpp_file(self, file):
        with open(file, mode="r") as stream:
            text = stream.read()

        original = text
        for method in (
            self.remove_empty_lines_after_curly_brace,
            self.remove_iostream_header,
            self.replace_platform_dependent_primitive_types,
            self.remove_std_primitive_types,
            self.replace_function_style_cast_to_primitive_types_by_static_cast,
            self.detect_c_style_cast,
            self.detect_fmt_print,
        ):
            try:
                text = method(text)
            except Exception:
                print(f"In file {file}:")
                raise

        if original != text:
            print("At least one check triggered on", file)
            if not self.dry_run:
                with open(file, mode="w") as stream:
                    stream.write(text)

    def get_list_of_files(self):
        return map(
            lambda x: pathlib.Path(x),
            subprocess.run(
                ["git", "ls-files"], cwd=REPO_DIR, check=True, stdout=subprocess.PIPE, text=True
            ).stdout.splitlines(),
        )

    def filter_cpp_files(self, files):
        return list(filter(lambda f: f.suffix in (".cpp", ".hpp", ".h"), files))

    def apply_all_rules_to_multiple_cpp_files(self, cpp_files):
        if self.verbose:
            for cpp_file in cpp_files:
                print(cpp_file)
                self.apply_all_rules_to_a_single_cpp_file(cpp_file)
        else:
            with multiprocessing.Pool(multiprocessing.cpu_count()) as pool:
                pool.map(self.apply_all_rules_to_a_single_cpp_file, cpp_files)


def parse_args():
    parser = argparse.ArgumentParser(description="Apply multiple project-specific rules")
    parser.add_argument(
        "-v",
        "--verbose",
        help="Print information on the rule-checking process (also disables parallel processing)",
        action="store_true",
    )
    parser.add_argument(
        "--dry-run",
        help="Run the script but do not change the files (also disabled parallel processing)",
        action="store_true",
    )
    return parser.parse_args()


if __name__ == "__main__":
    try:
        args = parse_args()
        app = CodeQualityRules(args.verbose, args.dry_run)
        files = app.get_list_of_files()
        cpp_files = app.filter_cpp_files(files)
        app.apply_all_rules_to_multiple_cpp_files(cpp_files)
        exit(0)
    except Exception as e:
        print(e)
        exit(1)
