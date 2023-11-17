#!/usr/bin/env python3

import json
import hashlib
import shutil
import subprocess
from pathlib import Path
from argparse import ArgumentParser, Namespace


def main():
    args = parse_args()

    features = probe_compiler_features(args)

    if args.cxx_standard:
        features['cxx_standard'] = args.cxx_standard

    with open(args.build_dependencies_file, encoding="utf8") as stream:
        build_dependencies = json.load(stream)

    for d in build_dependencies:
        download_dependency(d, args)

    if args.download_only:
        return

    for d in build_dependencies:
        build_dependency(d, args, features)

    print_prebuild_variables(args)


def parse_args():
    parser = ArgumentParser()
    parser.add_argument(
        "--project-dir",
        "-p",
        type=Path,
        help="root directory of this project, the one with README.md",
        default=Path(__file__).parent.parent.parent,
    )
    parser.add_argument(
        "--source-dir",
        "-S",
        type=Path,
        help="directory for the source of the dependencies, sharable by all bulid configurations",
        default=project_dir() / ".deps" / "source",
    )
    parser.add_argument(
        "--build-dir",
        "-B",
        type=Path,
        help="directory to build the dependencies, unique for each build configuration",
    )
    parser.add_argument(
        "--install-dir",
        "-i",
        type=Path,
        required=True,
        help="directory to install this configuration of this project and its dependencies",
    )
    parser.add_argument(
        "--build-dependencies-file",
        type=Path,
        help="file that defines the project dependencies and how to build them",
        default=Path(__file__).with_suffix(".json"),
    )
    parser.add_argument(
        "--build-type",
        "-c",
        type=str,
        help="value of CMAKE_BUILD_TYPE, defaults to RelWithDebInfo",
        default="RelWithDebInfo",
    )
    parser.add_argument(
        "--cxx-standard",
        type=int,
        help="set CMAKE_CXX_STANDARD, defaults to latest available C++ standard",
        default=None,
    )
    parser.add_argument(
        "--thread-count",
        "-j",
        type=str,
        help="limit the number of parallel processes when building dependencies",
    )
    parser.add_argument(
        "--download-only",
        "-d",
        help="when set stop processing after downloading the dependencies",
        action="store_true",
    )

    args = parser.parse_args()

    if not args.build_dir:
        args.build_dir = args.project_dir / ".deps" / "build" / unique_name(args)

    return args


def probe_compiler_features(args: Namespace) -> dict:
    features = {}

    source_dir = Path(__file__).parent / "probe_compiler_features"
    build_dir = args.build_dir / "probe_compiler_features"
    cmake_cache_file = build_dir / "CMakeCache.txt"

    run(["cmake", "-G", "Ninja", "-S", source_dir.as_posix(), "-B", build_dir.as_posix()])

    with open(cmake_cache_file) as stream:
        for line in stream.readlines():
            if line.startswith("HAVE_CXX_STANDARD:STRING="):
                features["cxx_standard"] = int(line.split("=")[1].rstrip())

    return features


def download_dependency(dep: dict, args: Namespace):
    source_dir = args.source_dir / qualified_name(dep)

    if not source_dir.is_dir():
        run(["git", "clone", dep["git_url"], "-b", dep["git_ref"], source_dir])


def build_dependency(dep: dict, args: Namespace, features: dict):
    source_dir = prepare_source_dir(dep, args)
    build_dir = args.build_dir / qualified_name(dep)
    install_dir = args.install_dir
    thread_count_a = ["-j", args.thread_count] if args.thread_count else []
    variables = [
        {"name": "CMAKE_CXX_STANDARD", "type": "STRING", "value": features["cxx_standard"]},
        {"name": "CMAKE_CXX_EXTENSIONS", "type": "BOOL", "value": "OFF"},
    ] + dep["variables"]

    cmake_configure(source_dir, build_dir, install_dir, args.build_type, variables)
    run(["ninja", "-C", build_dir] + thread_count_a)
    run(["ninja", "-C", build_dir, "install"])


def prepare_source_dir(dep: dict, args: Namespace):
    patch_dir = Path(__file__).parent / dep["name"]
    source_dir = args.source_dir / qualified_name(dep)

    if patch_dir.exists():
        print(f"Patch {dep['name']}")
        intermediate_dir = args.build_dir / f"{dep['name']}-src"
        shutil.copytree(
            source_dir, intermediate_dir, dirs_exist_ok=True, ignore=shutil.ignore_patterns(".*")
        )

        for file in patch_dir.glob("*"):
            shutil.copy2(file, intermediate_dir)

        return intermediate_dir
    else:
        return source_dir


def qualified_name(dep: dict):
    name = dep["name"]
    version = dep["git_ref"]
    return f"{name}-{version}"


def cmake_configure(
    source_dir: Path, build_dir: Path, install_dir: Path, build_type: str, variables: list
):
    args = ["cmake", "-G", "Ninja"]
    args += ["-S", source_dir.as_posix()]
    args += ["-B", build_dir.as_posix()]
    args.append(f"-DCMAKE_BUILD_TYPE={build_type}")
    args.append(f"-DCMAKE_INSTALL_PREFIX={install_dir.resolve().as_posix()}")

    for variable in variables:
        args.append(f"-D{variable['name']}={variable['value']}")

    run(args)


def run(args: list, cwd=None):
    args_s = [str(arg) for arg in args]
    cwd_s = f"cd {cwd} && " if cwd else ""
    print(f"> {cwd_s}{' '.join(args_s)}", flush=True)

    with subprocess.Popen(
        args_s, bufsize=1, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    ) as process:
        for line in process.stdout:
            print(line.rstrip(), flush=True)

        returncode = process.wait()

        if returncode != 0:
            raise RuntimeError(f"Sub-process exited with return code {returncode}")

        print()


def print_prebuild_variables(args):
    build_dir = f"build/{unique_name(args, 8)}"
    print("To configure and build this configuration of this project consider running:\n")
    print(
        f"    cmake -G Ninja -S . "
        f"-B {build_dir} "
        f"-DCMAKE_BUILD_TYPE={args.build_type} "
        f"-DCMAKE_INSTALL_PREFIX={args.install_dir.resolve().as_posix()}"
    )
    j_arg = f" -j {args.thread_count}" if args.thread_count else ""
    print(
        f"""    ninja -C {build_dir} {j_arg}
    ninja -C {build_dir} install

whereby \\ indicates line continuation and "{build_dir}" is a path that should be unique to this build configuration.

Make sure to run these commands from the same environment as the pre-build, e.g. using a x64 Native Tools Command Prompt"""
    )


def project_dir():
    return Path(__file__).parent.parent.parent


def unique_name(args, digits=32):
    hash_value = hashlib.md5(args.install_dir.resolve().as_posix().encode("utf8")).hexdigest()[
        :digits
    ]
    return f"{args.build_type}-{hash_value}"


if __name__ == "__main__":
    main()
