# Test Model for MPEG Immersive Video (TMIV)

[![pipeline status](http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1/badges/main/pipeline.svg)](http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1/-/commits/main)
[![coverage report](http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1/badges/main/coverage.svg)](http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1/-/graphs/main/charts)

## Introduction

This software project (TMIV-SW) is the ISO/IEC 23090-12 *MPEG Immersive Video* (MIV) reference software and is part of the *Test model for Immersive Video* [[4]](#references).

The MPEG-I project (ISO/IEC 23090) on *Coded Representation of Immersive Media* includes Part 2 *Omnidirectional MediA Format* (OMAF) version 1 published in 2018 that supports 3 Degrees of Freedom (3DoF), where a user’s position is static but its head can yaw, pitch and roll.
However, rendering flat 360° video, i.e. supporting head rotations only, may generate visual discomfort especially when objects close to the viewer are rendered. 6DoF enables translation movements in horizontal, vertical, and depth directions in addition to 3DoF orientations. The
translation support enables interactive motion parallax providing viewers with natural cues to their visual system and resulting in an enhanced perception of volume around them. At the 125th MPEG meeting, a call for proposals [[1]](#references) was issued to enable head-scale movements within a limited space. This has resulted in new MPEG-I Part 12 Immersive Video (MIV).

At the 129th MPEG meeting the fourth working draft of MIV has been realigned to use MPEG-I Part 5 *Video-based Point Cloud Compression* (V-PCC) as a normative reference for terms, defintions, syntax, semantics and decoding processes. At the 130th MPEG meeting this alignment has been completed by restructuring Part 5 in a common specification *Visual Volumetric Video-based Coding* (V3C) and annex H Video-based Point Cloud Compression (V-PCC). V3C provides extension mechanisms for V-PCC and MIV. The terminology in the test model reflects that of V3C and MIV.

## Scope

The normative decoding process for MPEG Immersive Video (MIV) is specified in *Text of ISO/IEC DIS 23090-12 MPEG Immersive Video* [[3]](#references). The latest public draft of the specification is *Potential improvements of MIV* [[2]](#references). The TMIV reference software provides a reference implementation of non-normative encoding and rendering techniques and the normative decoding process for the MIV
standard.

The test model document [[4]](#references) provides an algorithmic description for the TMIV encoder and decoder/renderer. The purpose of this document is to promote a common understanding of the coding features, in order to facilitate the assessment of the technical impact of new technologies during
the standardization process. *Common Test Conditions for MPEG Immersive Video* [[5]](#references) provides test conditions including TMIV-based anchors. Configuration files are included with the reference software.

## Build and installation instructions

TMIV is a C++17 CMake project that fetches all required (and optional, if requested) dependencies automatically.
For further build and installation instructions, see [doc/building.md](doc/building.md)

## Instructions to run TMIV

TMIV can be run in various CTC and non-CTC conditions, with various underlying 2D video codecs.
For details, see [doc/running.md](/doc/running.md).

## Configuring TMIV

TMIV's executables can be configured through configuration files and command line parameters.
For details, see [doc/configuring.md](/doc/configuring.md).

## Instruction to use TMIV as a library

In the `CMakeLists.txt` of the larger project, use the line:

```CMake
find_package(TMIV 7.0.0 REQUIRED)
```

The `TMIV_DIR` variable needs to be set to the directory named `/Workspace/tmiv_install/lib/cmake/TMIV` or `/Workspace/tmiv_install/lib64/cmake/TMIV` depending on the platform. 

After that the targets `TMIV::DecoderLib`, `TMIV::EncoderLib`, etc. are available as target dependencies.

## Structure of the test model

This software consists of multiple executables and static libraries. Below figure is the CMake module dependency graph of TMIV 5.0. The most important executables are Encoder and Decoder. When enabled, the project also includes the HM executables TAppEncoder and TAppDecoder.

![CMake module graph](doc/module_graph.svg)

Each module consists of one or more components. Most components derive from an interface and some interfaces have multiple alternative components. For instance, the ViewOptimizerLib includes an IViewOptimizer interface with NoViewOptimizer and ViewReducer components that implement that interface.

## Contributing to the test model

Core experiments are expected to include the reference software as a subproject
and introduce new components. Alternatively core experiments may branch the test
model. Contributions should be in the form of git merge requests to the
MPEG-internal repository. See [doc/contributing.md](doc/contributing.md) for further info.

## References

* [1] *Call for Proposals on 3DoF+ Visual*, ISO/IEC JTC 1/SC 29/WG 11 N 18145, Jan. 2019, Marrakesh, Morocco.
* [2] J. Boyce, R. Doré, V. Kumar Malamal Vadakital (Eds.), *Potential Improvements of MPEG Immersive Video*, [ISO/IEC JTC 1/SC 29/WG 04 N 0004](https://www.mpegstandards.org/wp-content/uploads/mpeg_meetings/132_OnLine/w19677.zip), October 2020, Online.
* [3] J. Boyce, B. Chupeau, L. Kondrad (Eds.), *Text of ISO/IEC DIS 23090-12 MPEG Immersive Video*, ISO/IEC JTC 1/SC 29/WG 04 N 0049, January 2021, Online.
* [4] B. Salahieh, C. Bachhuber, J. Jung, A. Dziembowski (Eds.), *Test Model 8 for MPEG Immersive Video*, [ISO/IEC JTC 1/SC 29/WG 04 N 0050](https://www.mpegstandards.org/wp-content/uploads/mpeg_meetings/133_OnLine/w20002.zip), January 2021, Online.
* [5] J. Jung, B. Kroon (Eds.), *Common Test Conditions for MPEG Immersive Video*, [ISO/IEC JTC 1/SC 29/WG 04 N 0051](https://www.mpegstandards.org/wp-content/uploads/mpeg_meetings/133_OnLine/w20003.zip), January 2021, Online.
