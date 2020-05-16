# Test Model for Immersive Video

## Introduction

This software project (TMIV-SW) is the ISO/IEC 23090-12 *MPEG Immersive Video* (MIV) 
reference software and is part of the *Test model for Immersive Video* [1].

The MPEG-I project (ISO/IEC 23090) on *Coded Representation of Immersive Media*
includes Part 2 *Omnidirectional MediA Format* (OMAF) version 1 published in 2018 that supports 3 
Degrees of Freedom (3DoF), where a user’s position is static but its head can yaw, pitch and roll.
However, rendering flat 360° video, i.e. supporting head rotations only, may generate visual 
discomfort especially when objects close to the viewer are rendered. 6DoF enables translation 
movements in horizontal, vertical, and depth directions in addition to 3DoF orientations. The 
translation support enables interactive motion parallax providing viewers with natural cues to their
visual system and resulting in an enhanced perception of volume around them. At the 125th MPEG 
meeting, a call for proposals [2] was issued to enable head-scale movements within a limited space. 
This has resulted in new MPEG-I Part 12 Immersive Video (MIV).

At the 129th MPEG meeting the fourth working draft of MIV has been realigned to use MPEG-I Part 5
*Video-based Point Cloud Compression* (V-PCC) as a normative reference for terms, defintions, syntax,
semantics and decoding processes. At the 130th MPEG meeting this alignment has been completed by
restructuring Part 5 in a common specification *Visual Volumetric Video-based Coding* (V3C) and annex H
Video-based Point Cloud Compression (V-PCC). V3C provides extension mechanisms for V-PCC and MIV.
The terminology in the test model reflects that of V3C and MIV.

## Scope

The normative decoding process for MPEG Immersive Video (MIV) is specified in
*Working Draft 5 of Immersive Video* [3]. The TMIV reference software provides a reference implementation
of non-normative encoding and rendering techniques and the normative decoding process for the MIV 
standard.

The test model document [1] provides an algorithmic description for the TMIV encoder and 
decoder/renderer. The purpose of this document is to promote a common understanding of the coding
features, in order to facilitate the assessment of the technical impact of new technologies during
the standardization process. *Common Test Conditions for Immersive Video* [4] provides test conditions
including TMIV-based anchors. Template configuration files are included with the reference software.

## Build and installation instructions

The software is ISO C++17 conformant and does not require external libraries. 
The optional dependencies are however highly recommended:

 *	Catch2 test framework
 *	HEVC test model (HM)
	
The following steps collect the software projects in a main working directory,
arbitrary called /Workspace in this description. Other directory names are also
not more than examples.

### Prerequisites to follow this instruction

Prerequisites are:

  * C++17 compiler and build tools
  * CMake 3.10 or newer
  * Git and SVN command tools

This description works for Windows and Linux. 

### Instructions to build and install Catch2

Catch2 is optional but highly recommended. Clone a recent version of
[Catch2](https://github.com/catchorg/Catch2.git) to enable the tests:

    cd /Workspace
    git clone https://github.com/catchorg/Catch2.git
    cd Catch2
    git checkout v2.11.1

Open the CMake GUI and specify:

  * Where is the source directory: /Workspace/Catch2
  * Where to build the binaries: /Workspace/Catch2/build
  * Click Configure, Yes, Finish
  * Set CMAKE_INSTALL_PREFIX to /Workspace/Catch2-2.11.1
  * Click Generate

Build and install the generated project.

### Instructions to download HM

Look in the CTC document for the version of HM and corresponding URL. This
description uses 16.16.

    cd /Workspace
    svn co https://hevc.hhi.fraunhofer.de/svn/svn_HEVCSoftware/tags/HM-16.16

TMIV includes a build script for HM.

### Instructions to build and install TMIV

To obtain the branch to this document:

    cd /Workspace
    git clone http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1.git
    cd TM1
    git checkout master
    
To obtain the latest release of TMIV instead:

    cd /Workspace
    git clone https://gitlab.com/mpeg-i-visual/tmiv.git
    
Open the CMake GUI and specify:

  * Where is the source directory: /Workspace/TM1
  * Where to build the binaries: /Workspace/TM1/build
  * Click Configure, Yes, Finish
  * Set CMAKE_INSTALL_PREFIX to /Workspace/TM1-master
  * Set Catch2_DIR to /Workspace/Catch2-v2.11.1/lib/cmake/Catch2
  * Set HM_SOURCE_DIR to /Workspace/HM-16.16
  * Click Generate

Build and install the generated project. 

For the Visual Studio CMake generators installation is performed by building the
INSTALL target. For Unix Makefiles CMake generators installation is through 
`make install`.

After this the TMIV executables Encoder and Decoder will be available under the
directory /Workspace/TM1-master/bin. By default TMIV only builds the HM modules
that are required for TMIV (TLibCommon and TLibDecoder). When
HM_BUILD_TAPPDECODER and HM_BUILD¬_TAPPENCODER are selected, then the
TAppDecoder and TAppEncoder tools respectively will also be installed to this
directory. 

## Instructions to run TMIV

Template configuration files are available under ctc_config/ and test_configs/. The file names of
and in template configuration files are examples.

  * *best_reference* uses all source views without coding to achieve the best possible result
  * *miv_anchor* is the MIV anchor with patches
  * *miv_view_anchor* is the MIV view anchor which codes a subset of views completely
  * *entity_based_coding_reference* is the (non-CTC) reference condition for entity-based coding
  * *entity_based_coding_test* is for integration testing of entity-based coding

Use the following steps to encode a bistream and render a viewport:

 1. Run TMIV encoder
 2. Run HM encoder on all video sub bitstreams
 3. Run TMIV multiplexer to form the output bitstream
 4. Run TMIV decoder to decode the bitstream and render a viewport

Use the following steps for uncoded video (i.e. best_reference):

 1. Run TMIV encoder
 4. Run TMIV decoder to render a viewport

## Structure of the test model

This software consists of multiple executables and static libraries. Below
figure is the CMake module dependency graph of TMIV 5.0. The most important
executables are Encoder and Decoder. When enabled, the project also includes the
HM executables TAppEncoder and TAppDecoder.

![CMake module graph](doc/module_graph.png)

Each module consists of one or more components. Most components derive from an
interface and some interfaces have multiple alternative components. For instance,
the ViewOptimizerLib includes an IViewOptimizer interface with NoViewOptimizer
and ViewReducer components that implement that interface.

## Improving the test model

Core experiments are expected to include the reference software as a subproject
and introduce new components. Alternatively core experiments may branch the test 
model. Contributions should be in the form of git pull requests to the
MPEG-internal repository.

## References

  * [1] *Call for Proposals on 3DoF+ Visual*, ISO/IEC JTC1/SC29/WG11 MPEG/N18145, Jan. 2019, Marrakesh,
Morocco.

  * [2] B. Salahihi, B. Kroon, J. Jung, A. Dziembowski (Eds.), *Test Model 5 for Immersive Video*,
ISO/IEC JTC1/SC29/WG11 MPEG/N19213, Apr. 2020, Online / Alpbach, Austria.

  * [3] J. Boyce, R. Doré, V. Kumar Malamal Vadakital (Eds.), *Working Draft 5 of Immersive Video*,
ISO/IEC JTC1/SC29/WG11 MPEG/N19212, Apr. 2020, Online / Alpbach, Austria.

  * [4] J. Jung, B. Kroon, J. Boyce, *Common Test Conditions for Immersive Video*, ISO/IEC JTC1/SC29/WG11
MPEG/N19214, Apr. 2020, Online / Alpbach, Austria.
