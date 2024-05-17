# MIV reference software

## Introduction

A general overview of ISO/IEC 23090-12 MPEG immersive video (MIV) is available at [mpeg-miv.org](http://mpeg-miv.org), including [a list of relevant MPEG documents](https://mpeg-miv.org/index.php/mpeg-documents/).

This software project provides:

* an embodiment of the *Test Model of MPEG immersive video* (TMIV)
* a ISO/IEC 23090-12 *MPEG immersive video* (MIV) codec
* the reference software encoder and decoder of ISO/IEC 23090-23 *Conformance and reference software for MPEG immersive video*.

## Getting started

For typical use there are the following requirements:

* C++17 or newer compiler toolchain
* Git
* Internet connection
* Python 3

The following dependencies are downloaded:

* Python packages: [requirements.txt](requirements.txt)
* C++ build dependencies: [build_dependencies.json](build_dependencies.json)

See [windows.md](doc/windows.md) and [ubuntu.md](doc/ubuntu.md) for installation instructions.

### Open a console

To get started, clone the project to a directory of choice and open a console in that location. For Microsoft Visual C++ it is important to open an *x64 Native Tools Command Prompt*. The compiler toolchain needs to be accessible from the console. If unsure test this by running `g++ --version`, `clang++ --version` or `cl.exe` in the console and check if the version is as expected.

### Create a Python virtual environment

```shell
python -m venv venv
. venv/bin/activate    # On Linux
venv\Scripts\activate  # On Windows
python -m pip install --upgrade pip
pip install -r requirements.txt
```

### Install the project

The following line downloads external dependencies, builds them, builds this project, runs unit tests and install this project, for a given CMake preset `PRESET`:

```shell
python scripts/install.py PRESET  # Try --help for more options
```

Run `cmake --list-presets` for a list of available presets.

### CMake presets

 The provided presets are for three popular compilers and three build types:

| Compiler | Debug | RelWithDebInfo | Release | Conditional availability |
|---------|-------|---------------|-----------|----------------|
| [Clang (LLVM)](https://github.com/llvm/llvm-project/) | `clang-debug` | `clang-develop` | `clang-release` | Linux |
| [GNU Compiler Collection (GCC)](https://gcc.gnu.org/) | `gcc-debug` | `gcc-develop` | `gcc-release` | Linux |
| [Microsoft Visual C++ (MSVC)](https://visualstudio.microsoft.com/) | `cl-debug` | `cl-develop` | `cl-release` | x64 Native Tools Command Prompt |

For normale use (coding experiments), select one of the `*-release` presets.

### Running the software

TMIV can be run in various CTC and non-CTC conditions, with various underlying 2D video codecs.
For details, see [running.md](/doc/running.md).

TMIV's executables can be configured through configuration files and command line parameters.
For details, see [configuring.md](/doc/configuring.md).

### Advanced use

* TMIV is a CMake project and it can be built without a script or CMake presets.
* The `install.py` script supports custom presets in a `CMakeUserPresets.json` file like CMake does.
* Run the `install.py` script without arguments to only download external dependencies.
* None of the build dependencies are required if the C++ compiler supports the `<print>` header.
* TMIV can be used as a library in a super project.

## Contributing

Contributions should be in the form of GitLab merge requests to the MPEG-internal repository.

### Development presets and scripts

For regular development work select one of the `*-develop` presets. The `*-debug` presets build faster but run much slower.

Before pushing draft code to the MPEG Git server, check formatting and coding guidelines:

```shell
python scripts/check.py           # Try --help for more options
```

Before marking a merge request as ready, run all unit, integration and conformance tests:

```shell
python scripts/test.py PRESET     # Try --help for more options
```

It is advised to run the tests both on the parent branch and the development branch and with multiple presets. Some generated MD5 sums are compiler-specific due to floating point use. The required test data is available on the MPEG content server at [/MPEG-I/Part12-ImmersiveVideo/tmiv_integration_test](https://mpegfs.int-evry.fr/mpegcontent/ws-mpegcontent/MPEG-I/Part12-ImmersiveVideo/tmiv_integration_test). By default the script will expect the test data in the `in/test` directory.

### Structure of the reference software

This software consists of multiple executables and static libraries. Below figure is the CMake module dependency graph. The most important executables are Encoder and Decoder. When enabled, the project also includes the HM executables TAppEncoder and TAppDecoder.

![CMake module graph](doc/module_graph.svg)

Each module consists of one or more components. Most components derive from an interface and some interfaces have multiple alternative components. For instance, the ViewOptimizerLib includes an IViewOptimizer interface with NoViewOptimizer and ViewReducer components that implement that interface.
