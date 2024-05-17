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
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def main(
    preset_names: list[str],
    skip_download: bool,
    skip_prebuild: bool,
    skip_configure: bool,
    remove_cmake_cache: bool,
    skip_build: bool,
    skip_tests: bool,
    skip_install: bool,
    build_dependencies_file: Path,
    dependencies_source_dir: Path,
    project_dir: Path,
    thread_count: int,
):
    build_dependencies = (
        None
        if skip_download and (skip_prebuild or not preset_names)
        else load_json(build_dependencies_file)
    )

    if not skip_download:
        download_dependencies(build_dependencies, dependencies_source_dir)
        patch_dependencies(build_dependencies, dependencies_source_dir, project_dir)

    presets = load_presets(project_dir) if preset_names else None

    if not preset_names:
        print("No presets were specified. Try --help")

    for preset_name in preset_names:
        preset = resolve_preset(preset_name, presets, project_dir)

        features = (
            None
            if skip_prebuild and skip_configure
            else probe_compiler_features(preset, project_dir)
        )

        if not skip_prebuild:
            prebuild(
                build_dependencies,
                preset,
                features,
                remove_cmake_cache,
                dependencies_source_dir,
                thread_count,
            )

        if not skip_configure:
            configure(preset, features, project_dir, remove_cmake_cache)

        build_dir = preset["binaryDir"]

        if not skip_build:
            build(build_dir, thread_count)

        if not skip_tests:
            test(build_dir)

        if not skip_install:
            install(build_dir)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="Download dependencies, prebuild, build, test and install TMIV.",
        epilog="Run this script from a suitable Python virtual environment, "
        "and also make sure that the environment is set-up for the intended compiler toolchain. "
        "For Microsoft Visual Studio, run in a x64 Native Tools Command Prompt.",
    )

    parser.add_argument(
        "preset_names",
        type=str,
        nargs="*",
        help="List of CMake configure presets for which to prebuild, build, test and install TMIV. "
        "Specifying none will still download dependencies. "
        "Use cmake --list-presets for a list of available presets.",
        default=[],
    )

    parser.add_argument(
        "--skip-download",
        action="store_true",
        help="Skip downloading of build dependencies",
    )
    parser.add_argument(
        "--skip-prebuild",
        action="store_true",
        help="Skip building of build dependencies",
    )
    parser.add_argument(
        "--skip-configure",
        action="store_true",
        help="Skip build configuration of TMIV",
    )
    parser.add_argument(
        "--remove-cmake-cache",
        action="store_true",
        help="Remove CMakeCache.txt when configuration TMIV or its build dependencies.",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Skip building of TMIV",
    )
    parser.add_argument(
        "--skip-tests",
        action="store_true",
        help="Skip testing of TMIV",
    )
    parser.add_argument(
        "--skip-install",
        action="store_true",
        help="Skip installation of TMIV",
    )
    parser.add_argument(
        "--build-dependencies-file",
        type=Path,
        help="file that defines the project dependencies and how to build them (relative to --project-dir)",
        default=Path("build_dependencies.json"),
    )
    parser.add_argument(
        "-S",
        "--dependencies-source-dir",
        type=Path,
        help="directory for the source of the dependencies, sharable by all bulid configurations (relative to --project-dir)",
        default=Path(".deps") / "source",
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
        help="limit the number of parallel processes when building dependencies",
        default=0,
    )

    args = parser.parse_args()

    args.build_dependencies_file = args.project_dir / args.build_dependencies_file
    args.dependencies_source_dir = args.project_dir / args.dependencies_source_dir

    return args


def load_presets(project_dir: Path) -> dict:
    result = dict()

    for file in [project_dir / "CMakePresets.json", project_dir / "CMakeUserPresets.json"]:
        if file.is_file():
            with open(file) as stream:
                result.update({x["name"]: x for x in json.load(stream).get("configurePresets", [])})

    return result


PRESET_PATH_KEYS = ("binaryDir", "installDir")


def resolve_preset(preset_name: str, presets: dict, project_dir: Path = None) -> dict:
    result = presets[preset_name]

    if "inherits" in result:
        inherits = result["inherits"]

        for base_name in [inherits] if isinstance(inherits, str) else inherits:
            result = inherit(result, resolve_preset(base_name, presets))

        del result["inherits"]

    if project_dir:
        for key in PRESET_PATH_KEYS:
            result[key] = result[key].replace("${presetName}", preset_name)
            result[key] = result[key].replace("${sourceDir}", project_dir.as_posix())
            result[key] = Path(result[key]).resolve().as_posix()

        print("Resolved CMake configuration preset:", json.dumps(result, indent=4, sort_keys=True))

        for key in PRESET_PATH_KEYS:
            result[key] = Path(result[key])

    return result


def inherit(derived: dict, base: dict) -> dict:
    result = dict(derived)

    for key, value in base.items():
        if key in ["name", "hidden", "inherits", "description", "displayName", "condition"]:
            pass
        elif not key in derived:
            result[key] = value
        elif isinstance(value, dict):
            result[key] = inherit(derived[key], value)

    return result


def prebuild(
    build_dependencies: dict,
    preset: dict,
    features: dict,
    remove_cmake_cache: bool,
    dependencies_source_dir: Path,
    thread_count: int,
):
    if 202207 <= features["__cpp_lib_format"] and 202207 <= features["__cpp_lib_print"]:
        build_dependencies = [d for d in build_dependencies if d["name"] != "FMT"]

    for dependency in build_dependencies:
        build_dependency(
            dependency, preset, features, remove_cmake_cache, dependencies_source_dir, thread_count
        )


def probe_compiler_features(preset: dict, project_dir: Path) -> dict:
    source_dir = project_dir / "cmake" / "probe_compiler_features"
    build_sub_dir = preset["binaryDir"] / "probe_compiler_features-build"
    install_dir = preset["binaryDir"] / "probe_compiler_features-install"
    compiler_features_file = preset["binaryDir"] / "compiler_features.json"

    run(
        ["cmake", "-G", "Ninja"]
        + ["-S", source_dir.as_posix()]
        + ["-B", build_sub_dir.as_posix()]
        + ["--install-prefix", install_dir.as_posix()]
    )
    run(["ninja", "-C", build_sub_dir])
    run(["ninja", "-C", build_sub_dir, "install"])
    run([install_dir / "bin" / "probe_compiler_features", compiler_features_file])

    with open(compiler_features_file) as stream:
        features = json.load(stream)

    print("Probed C++ compiler features:", json.dumps(features, indent=4))

    return features


def download_dependencies(build_dependencies: list, dependencies_source_dir: Path):
    for dependency in build_dependencies:
        source_dir = dependencies_source_dir / qualified_name(dependency)

        if not source_dir.is_dir():
            run(["git", "clone", dependency["git_url"], "-b", dependency["git_ref"], source_dir])


def patch_dependencies(build_dependencies: list, dependencies_source_dir: Path, project_dir: Path):
    for dependency in build_dependencies:
        patch_dir = project_dir / "cmake" / dependency["name"]

        if patch_dir.exists():
            print(f"Patching {dependency['name']}")
            source_dir = dependencies_source_dir / qualified_name(dependency)

            for file in patch_dir.glob("*"):
                shutil.copy2(file, source_dir)


def build_dependency(
    dependency: dict,
    preset: dict,
    features: dict,
    remove_cmake_cache: bool,
    dependencies_source_dir: Path,
    thread_count: int,
):
    source_dir = dependencies_source_dir / qualified_name(dependency)
    build_dir = preset["binaryDir"] / ".deps" / qualified_name(dependency)
    install_dir = preset["installDir"]

    variables = {
        "CMAKE_CXX_STANDARD": features["cxx_standard"],
        "CMAKE_BUILD_TYPE": preset["cacheVariables"]["CMAKE_BUILD_TYPE"],
    }
    variables.update(dependency["variables"])

    cmake_configure(source_dir, build_dir, install_dir, variables, remove_cmake_cache)
    build(build_dir, thread_count)
    install(build_dir)


def configure(preset: dict, features: dict, project_dir: Path, remove_cmake_cache: bool):
    variables = {"CMAKE_CXX_STANDARD": features["cxx_standard"]}
    variables.update(preset["cacheVariables"])

    cmake_configure(
        project_dir, preset["binaryDir"], preset["installDir"], variables, remove_cmake_cache
    )


def cmake_configure(
    source_dir: Path, build_dir: Path, install_dir: Path, variables: dict, remove_cmake_cache: bool
):
    if remove_cmake_cache:
        cache_file = build_dir / "CmakeCache.txt"

        if cache_file.exists():
            print(f"Removing {cache_file}")
            cache_file.unlink()

    args = ["cmake", "-G", "Ninja"]
    args += ["-S", source_dir.as_posix()]
    args += ["-B", build_dir.as_posix()]
    args += ["--install-prefix", install_dir.as_posix()]

    for name, value in variables.items():
        args.append(f"-D{name}={value}")

    run(args)


def build(build_dir: Path, thread_count: int):
    thread_count_a = ["-j", thread_count] if thread_count else []

    run(["ninja", "-C", build_dir] + thread_count_a)


def test(build_dir: Path):
    run(["ctest", "--output-on-failure"], cwd=build_dir)


def install(build_dir: Path):
    run(["ninja", "-C", build_dir, "install"])


def load_json(file: Path) -> dict:
    with open(file, encoding="utf8") as stream:
        return json.load(stream)


def qualified_name(dep: dict):
    name = dep["name"]
    version = dep["git_ref"]
    return f"{name}-{version}"


def unique_name(args, digits=32):
    hash_value = hashlib.md5(args.install_dir.resolve().as_posix().encode("utf8")).hexdigest()[
        :digits
    ]
    return f"{args.build_type}-{hash_value}"


def run(args: list, cwd=None):
    args_s = [str(arg) for arg in args]
    cwd_s = f"cd {cwd} && " if cwd else ""
    print(f"# {cwd_s}{' '.join(args_s)}", flush=True)

    with subprocess.Popen(
        args_s, bufsize=1, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    ) as process:
        have_output = False

        for line in process.stdout:
            print(line.rstrip(), flush=True)
            have_output = True

        returncode = process.wait()

        if returncode != 0:
            raise RuntimeError(f"Sub-process exited with return code {returncode}")

        if have_output:
            print()


if __name__ == "__main__":
    args = parse_arguments()
    main(**vars(args))
