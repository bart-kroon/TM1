# Build instructions

These instructions will result in a source directory `/Workspace/tmiv`, a build directory `/Workspace/tmiv_build` and an installation directory `/Workspace/tmiv_install`. Directory names in this instruction are merely examples.

The workflow in general has the following steps:

1. Satisfy the prerequisites
2. Obtain the project
3. Satisfy the project dependencies
4. Configure the project
5. Build and install the project

## Prerequisites

The typical system requirements to build this software are:

- A terminal or integrated development environment (IDE)
- [C++17](https://en.cppreference.com/w/cpp/17) compatible compiler toolchain such as a recent version of [Visual Studio](https://visualstudio.microsoft.com/), [GCC](https://duckduckgo.com/?q=gnu+gcc&ia=web) or [Clang](https://clang.llvm.org/)
- [CMake](https://cmake.org/) 3.14 or newer
- [Ninja](https://ninja-build.org/) 1.10 or newer
- [Git](https://git-scm.com/) command-line tools
- [Python](https://www.python.org/) 3.8 or newer
- Internet connection

The minimal system requirements are the terminal, C++17 compiler toolchain and CMake.

## Project location

This project is available at the following locations:

- MPEG-internal repository: [http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1.git](http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1.git)
- Public mirror: [https://gitlab.com/mpeg-i-visual/tmiv.git](https://gitlab.com/mpeg-i-visual/tmiv.git)

To obtain release *x*.*y* of this project run:

```shell
git clone https://gitlab.com/mpeg-i-visual/tmiv.git -b vx.y /Workspace/tmiv
```

Alternatively, to obtain the development branch run:

```shell
git clone http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1.git /Workspace/tmiv
```

Alternatively, use an IDE or download a zip-file of the project and unpack at `/Workspace/tmiv` such that `/Workspace/tmiv/README.md` exists.

## Project dependencies

This project has the following dependencies, with suitable versions listed in [build_dependencies.json](../scripts/build/build_dependencies.json):

| project                                          | description                   | requirement                              |
| ------------------------------------------------ | ----------------------------- | ---------------------------------------- |
| [{fmt}](https://github.com/fmtlib/fmt)           | library for string formatting | required to build                        |
| [Catch2](https://github.com/catchorg/Catch2.git) | testing framework             | required to build tests                  |
| [HM](https://vcgit.hhi.fraunhofer.de/jvet/HM)    | HEVC test model               | optional HEVC support                    |
| [VVdeC](https://github.com/fraunhoferhhi/vvdec)  | VVdeC VVC decoder             | optional VVC support                     |
| [VVenC](https://github.com/fraunhoferhhi/vvenc)  | VVenC VVC encoder             | runtime dependency for integration tests |

Obtain and build the project dependencies using the prebuild script provided by this project:

```shell
python /Workspace/tmiv/scripts/build/build_dependencies.py -i /Workspace/tmiv_install
```

Wherein `python` is the name of the Python 3 interpreter. Use the same installation directory for the dependencies and this project itself. When working with multiple build types or compilers, use a unique directory for each.

For Visual Studio there is a batch-file that will prepare the environment for a 64-bit native build. All arguments are forwarded to the Python script. To use the default install locations for this IDE run:

```batch
cd /Workspace/tmiv
scripts/build/build_dependencies.bat -i out/install/x64-Release -c RelWithDebInfo
scripts/build/build_dependencies.bat -i out/install/x64-Debug -c Debug
```

- Use `--help` to obtain a list of available arguments
- Use `--build-type` to specify the [build type](https://cmake.org/cmake/help/latest/variable/CMAKE_BUILD_TYPE.html). By default, builds are performed using the `RelWithDebInfo` build type.
- Use `--download-only` to download one one machine, copy the entire `/Workspace/tmiv` directory to another machine, and continue prebuild on that machine. This is useful for compute nodes with limited internet access.

Use of the prebuild script is preferred but optional. Alternatively, prebuild the dependencies by means outside of the scope of these instructions.

## Project configuration

To configure the project execute:

```shell
cmake \
    -G Ninja -S /Workspace/tmiv -B /Workspace/tmiv_build \
    -DCMAKE_INSTALL_PREFIX=/Workspace/tmiv_install \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Alternatively, use an IDE with CMake integration:

- Open the `/Workspace/tmiv` folder in the IDE
- Edit the CMake settings
- Generate the CMake project

This project expects prebuilt libraries in default locations. For alternate locations specify the `FMT_DIR`, `Catch2_DIR`, `HM_DIR` and `vvdec_DIR` variables.

## Building the project

To build and install the project execute:

```shell
ninja -C /Workspace/tmiv_build
```

Type `ninja --help` for available options. By default `ninja` uses parallel processing. Use `-j` to specify the number of concurrent build steps.

Alternatively, use an IDE to build the project.

## Installing the project

To install the project execute:

```shell
ninja -C /Workspace/tmiv_build install
```
