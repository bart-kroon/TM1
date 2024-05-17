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
import json
from pathlib import Path
import re
import subprocess

from install import run

MAX_COMPACT_LIST_LEN = 200
INDENTING = "    "


def main(
    project_dir: Path,
    skip_cpp_formatting: bool,
    cpp_formatter: Path,
    skip_cmake_formatting: bool,
    cmake_formatter: Path,
    python_formatter: Path,
    skip_python_formatting: bool,
    skip_json_formatting: bool,
    skip_code_quality_rules: bool,
):
    if not skip_cpp_formatting:
        format_cpp(project_dir, cpp_formatter)

    if not skip_cmake_formatting:
        format_cmake(project_dir, cmake_formatter)

    if not skip_python_formatting:
        format_python(project_dir, python_formatter)

    if not skip_json_formatting:
        format_json(project_dir)

    if not skip_code_quality_rules:
        apply_code_quality_rules(project_dir)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="Apply formatting and code quality checks",
        epilog="Run this script before pushing to a branch.",
    )

    parser.add_argument(
        "--skip-cpp-formatting",
        action="store_true",
        help="Skip C++ formatting",
    )
    parser.add_argument(
        "--cpp-formatter",
        type=Path,
        default=Path("clang-format"),
        help="C++ formatter executable",
    )
    parser.add_argument(
        "--skip-cmake-formatting",
        action="store_true",
        help="Skip CMake formatting",
    )
    parser.add_argument(
        "--cmake-formatter",
        type=Path,
        default=Path("cmake-format"),
        help="CMake formatter executable",
    )
    parser.add_argument(
        "--skip-python-formatting",
        action="store_true",
        help="Skip Python formatting",
    )
    parser.add_argument(
        "--python-formatter",
        type=Path,
        default=Path("black"),
        help="Python formatter executable",
    )
    parser.add_argument(
        "--skip-json-formatting",
        action="store_true",
        help="Skip CMake formatting",
    )
    parser.add_argument(
        "--skip-code-quality-rules",
        action="store_true",
        help="Skip code quality rules",
    )
    parser.add_argument(
        "--project-dir",
        type=Path,
        help="TMIV source directory (the directory with the README.md)",
        default=Path(__file__).parent.parent.resolve(),
    )
    return parser.parse_args()


def format_cpp(project_dir: Path, cpp_formatter: Path):
    for files in selected_project_files_by_dir(
        project_dir, lambda f: f.suffix in (".cpp", ".hpp", ".h")
    ).values():
        run([cpp_formatter, "-style=File", "-i"] + files)


def format_cmake(project_dir: Path, cmake_formatter: Path):
    for files in selected_project_files_by_dir(
        project_dir, lambda f: f.suffix == ".cmake" or f.name == "CMakeLists.txt"
    ).values():
        run([cmake_formatter, "-i"] + files)


def format_python(project_dir: Path, python_formatter: Path):
    for files in selected_project_files_by_dir(project_dir, lambda f: f.suffix == ".py").values():
        run([python_formatter] + files)


def format_json(project_dir: Path):
    for files in selected_project_files_by_dir(project_dir, lambda f: f.suffix == ".json").values():
        for file in files:
            format_json_file(file)


def format_json_file(file):
    print(f"Formatting {file}")

    with open(file, encoding="utf8") as stream:
        original = stream.read()

    data = json.loads(original)
    text = format_json_node(data) + "\n"

    if original != text:
        with open(file, mode="w", encoding="utf8") as stream:
            stream.write(text)


def format_json_node(data, indent="") -> str:
    if isinstance(data, dict) and len(data) > 0:
        return format_json_dict(data, indent)
    else:
        text = json.dumps(data, sort_keys=True)

        if len(text) <= MAX_COMPACT_LIST_LEN:
            return text

        if isinstance(data, list):
            return format_json_list(data, indent)
        else:
            return json.dumps(data)


def format_json_list(data: list, indent="") -> str:
    sub_indent = indent + INDENTING
    sub = ",\n".join(map(lambda value: f"{sub_indent}{format_json_node(value, sub_indent)}", data))
    sub = re.sub(r"(\s+)\}\,\n\s+\{\n", r"\1}, {\n", sub)
    return f"[\n{sub}\n{indent}]"


def format_json_dict(data: dict, indent="") -> str:
    sub_indent = indent + INDENTING
    sub = ",\n".join(
        map(
            lambda key: f'{sub_indent}"{key}": {format_json_node(data[key], sub_indent)}',
            sorted(data),
        )
    )
    return f"{{\n{sub}\n{indent}}}"


def apply_code_quality_rules(project_dir: Path):
    for files in selected_project_files_by_dir(
        project_dir, lambda f: f.suffix in (".cpp", ".hpp", ".h")
    ).values():
        for file in files:
            apply_code_quality_rules_to_file(file)


def apply_code_quality_rules_to_file(file: Path):
    print(f"Applying code quality rules to {file}")

    with open(file, mode="r") as stream:
        text = stream.read()

    original = text
    for method in (
        remove_empty_lines_after_curly_brace,
        remove_iostream_header,
        replace_platform_dependent_primitive_types,
        remove_std_primitive_types,
        replace_function_style_cast_to_primitive_types_by_static_cast,
        detect_c_style_cast,
        detect_fmt_print,
    ):
        text = method(text)

    if original != text:
        with open(file, mode="w") as stream:
            stream.write(text)


def remove_empty_lines_after_curly_brace(text: str):
    return text.replace("{\n\n", "{\n")


def remove_iostream_header(text: str):
    return text.replace("#include <iostream>\n", "")


def replace_platform_dependent_primitive_types(text: str):
    return re.sub(
        r"(//[^\n]+|main)?[^a-zA-Z0-9_\"]((un)?signed\s+)?(short|int|long|long long)[^a-zA-Z0-9_\"]",
        replace_platform_dependent_types_replace,
        text,
    )


def replace_platform_dependent_types_replace(match):
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


def remove_std_primitive_types(text: str):
    return re.sub(
        r"std::(u?int(_fast|_least)?(8|16|32|64)_t|u?intmax_t|u?intptr_t|size_t|ptrdiff_t)[^a-zA-Z0-9_]",
        remove_std_primitive_types_replace,
        text,
    )


def remove_std_primitive_types_replace(match):
    return match[0][5:]


def replace_function_style_cast_to_primitive_types_by_static_cast(text: str):
    return re.sub(
        r"(//[^\n]+)?(operator|function)?([^a-z0-9_:])(nullptr_t|signed|unsigned|short|long|int|bool|char|float|double|u?int(_fast|_least)?(8|16|32|64)_t|u?intmax_t|u?intptr_t|size_t|ptrdiff_t|streamoff)\(",
        replace_function_style_cast_to_primitive_types_by_static_cast_replace,
        text,
    )


def replace_function_style_cast_to_primitive_types_by_static_cast_replace(match):
    if match[1] or match[2]:
        return match[0]
    return "{0}static_cast<{1}>(".format(match[3], match[4])


def detect_c_style_cast(text: str):
    return re.sub(
        r"(//[^\n]+)?\((bool|char|float|double|u?int(_fast|_least)?(8|16|32|64)_t|u?intmax_t|u?intptr_t|size_t|ptrdiff_t|streamoff)\)[^\)>]",
        detect_c_style_cast_replace,
        text,
    )


def detect_c_style_cast_replace(match):
    if match[1]:
        return match[0]

    type_ = match[0][1:-1]
    raise RuntimeError(
        f"Replace c-style cast {match[0]} with Common::downCast<{type_}>( ) or static_cast<{type_}>( )"
    )


def detect_fmt_print(text: str):
    return re.sub(r"::print\(\"", detect_fmt_print_replace, text)


def detect_fmt_print_replace(match):
    raise RuntimeError(
        "Replace `TMIV_FMT::print(fmt, args...)` with `Common::logInfo(fmt, args...)`. "
        "Note that `TMIV_FMT::print(stream, fmt, args...)` with `std::ostringstream &stream` is allowed."
    )


def selected_project_files_by_dir(project_dir: Path, predicate) -> dict[Path, list[Path]]:
    result: dict[Path, list[Path]]
    result = dict()

    for line in subprocess.check_output(["git", "ls-files"], cwd=project_dir).splitlines():
        file = project_dir / str(line, "utf-8")

        if predicate(file):
            dir = file.parent

            if not dir in result:
                result[dir] = []

            result[dir].append(file)

    return result


if __name__ == "__main__":
    args = parse_arguments()
    main(**vars(args))
