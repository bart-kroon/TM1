# Build and installation instructions

Directory names in this instruction are merely examples.

## Prerequisites to follow this instruction

These instructions are valid for Windows and Linux.

The software is ISO C++17 conformant and requires the following external libraries:

* The [{fmt}](https://github.com/fmtlib/fmt) library for string formatting
* The [Catch2](https://github.com/catchorg/Catch2.git) test framework
* The [HEVC test model](https://vcgit.hhi.fraunhofer.de/jct-vc/HM.git) (HM)

Minimal prerequisites are:

* C++17 compiler and build tools
* CMake 3.14 or newer

The preferred method is to let the TMIV build configuration process download the correct versions of all the external libraries for you. This adds the following prerequisites:
* Git command-line tools
* Internet connection

For the fallback procedure, please continue reading at [Preparing for TMIV build on a machine without internet connection](#preparing-for-tmiv-build-on-a-machine-without-internet-connection).

## Obtain a version of TMIV

To obtain the latest public release of TMIV:

```shell
cd /Workspace
git clone https://gitlab.com/mpeg-i-visual/tmiv.git
```

To obtain the latest internal release of TMIV instead:

```shell
cd /Workspace
git clone http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1.git tmiv
```

The default branch of the internal repository is the current development branch, e.g. `v7.0-dev`.

To switch on either server to a specific release of TMIV, e.g. 6.1, also execute the following command:

```shell
cd /Workspace/tmiv
git checkout v6.1
```

Please continue reading at [Building and installing TMIV](#building-and-installing-tmiv).

## Preparing for TMIV build on a machine without internet connection

The first step is to obtain source archives for TMIV and all external libraries on a machine with internet access.

### Catch2 testing framework

1. Visit this URL: https://github.com/catchorg/Catch2/tree/v2.13.4
1. Click on the green button "Code" and select "Download ZIP"
1. Unzip, resulting in a directory `/Workspace/Catch2-2.13.4` such that the file `/Workspace/Catch2-2.13.4/README.md` exists.

### {fmt} string formatting library

1. Visit this URL: https://github.com/fmtlib/fmt/tree/7.0.3
1. Click on the green button "Code" and select "Download ZIP"
1. Unzip, resulting in a directory `/Workspace/fmt-7.0.3` such that the file `/Workspace/fmt-7.0.3/README.md` exists.

### HEVC test model (HM)

1. Visit this URL: https://vcgit.hhi.fraunhofer.de/jct-vc/HM/-/tree/HM-16.16
1. Click on the download button (next to the Clone button) and select "zip"
1. Unzip, resulting in a directory `/Workspace/HM-HM-16.16` such that the file `/Workspace/HM-HM-16.16/README` exists.
1. Rename that directory to `/Workspace/HM-16.16`.

### Fraunhofer Versatile Video Encoder (VVenC)

1. Visit this URL: https://github.com/fraunhoferhhi/vvenc/tree/v0.2.0.0
1. Click on the green "Code" button and select "Download ZIP"
1. Unzip, resulting in a directory `/Workspace/vvenc-0.2.0.0` such that the file `/Workspace/vvenc-0.2.0.0/README.md` exists.

### Fraunhofer Versatile Video Decoder (VVdeC)

1. Visit this URL: https://github.com/fraunhoferhhi/vvdec/tree/v1.0.1
1. Click on the green "Code" button and select "Download ZIP"
1. Unzip, resulting in a directory `/Workspace/vvdec-1.0.1` such that the file `/Workspace/vvdec-1.0.1/README.md` exists.

### This project

1. For the latest public release, visit this URL: https://gitlab.com/mpeg-i-visual/tmiv
1. Click on the download button (next to the Clone button) and select "zip"
1. Unzip, resulting in a directory `/Workspace/tmiv-main` such that the file `/Workspace/tmiv-main/README.md` exists.
1. Rename to `/Workspace/tmiv` to match with the following instructions.

To select a specific release instead, e.g. 6.1, use this procedure instead:

1. Visit this URL: https://gitlab.com/mpeg-i-visual/tmiv/-/tree/v6.1
1. Click on the download button (next to the Clone button) and select "zip"
1. Unzip, resulting in a directory `/Workspace/tmiv-v6.1` such that the file `/Workspace/tmiv-v6.1/README.md` exists.
1. Rename to `/Workspace/tmiv` to match with the following instructions.

The same instruction also works with the internal software repository. Pleae be aware, that the default branch of the internal repository is not `main` but the current development branch, e.g. `v7.0-dev`. This is not a stable version of TMIV.

In the next step add `-DNO_INTERNET=ON` to the first cmake command or set `NO_INTERNET` to `ON` in the CMake GUI. If the external libraries are placed in the right directories, then they are detected. Otherwise the location of each of the external libraries can be specified, for instance by entering the values in the CMake GUI.

## Building and installing TMIV

Below are two alternative instructions for building: using only command-line tools or using a GUI. Both instructions are compatible with Linux and Windows. Make sure to use forward slashes with CMake, also on Windows.

### Using the command line

To build and install TMIV into the directory `/Workspace/tmiv_install` with sources in the directory `/Workspace/tmiv`, execute:

```shell
mkdir /Workspace/tmiv_build
cd /Workspace/tmiv_build
cmake -DCMAKE_INSTALL_PREFIX=/Workspace/tmiv_install -DCMAKE_BUILD_TYPE=Release /Workspace/tmiv
cmake --build . --parallel --config Release
cmake --build . --parallel --config Release --target install
```

The intermediate directory `/Workspace/tmiv_build` may be deleted when the installation has completed.

### Using a GUI

Open the CMake GUI and specify:

* Where the source directory is: `/Workspace/tmiv`
* Where to build the binaries: `/Workspace/tmiv_build`
* Click Configure, Yes, Finish
* Do not change any flags starting with `CATCH_`
* Set `CMAKE_INSTALL_PREFIX` to `/Workspace/tmiv_install`
* Set `CMAKE_BUILD_TYPE` to `Release` (no need for Visual Studio)
* Click Generate

Build and install the generated project.

For Visual Studio please:
* Manually select `Release` from the drop-down box.
* Perform build by building the `ALL_BUILD` target.
* Perform installation by building the `INSTALL` target.

## Installation result

After installation, the TMIV executables `Encoder`, `Decoder` and `Renderer` will be available under the directory `/Workspace/tmiv_install/bin`.
By default TMIV only builds the HM modules that are required for TMIV (`TLibCommon` and `TLibDecoder`).
When `HM_BUILD_TAPPDECODER` and `HM_BUILD_TAPPENCODER` are selected, then the `TAppDecoder` and `TAppEncoder` tools respectively will also be installed to this directory.
TMIV per default builds `vvencFFapp` and `vvdecapp` from the Fraunhofer VVC implementations.
