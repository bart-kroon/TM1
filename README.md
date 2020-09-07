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
*Text of ISO/IEC CD 23090-12 MPEG Immersive Video* [3]. The TMIV reference software provides a reference implementation
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

* Catch2 test framework
* HEVC test model (HM)

The following steps collect the software projects in a main working directory,
arbitrary called /Workspace in this description. Other directory names are also
not more than examples.

### Prerequisites to follow this instruction

Prerequisites are:

* C++17 compiler and build tools
* CMake 3.10 or newer
* Git

This description works for Windows and Linux.

### Instructions to build and install Catch2

Catch2 is optional but highly recommended. Clone a recent version of
[Catch2](https://github.com/catchorg/Catch2.git) to enable the tests:

```shell
cd /Workspace
git clone https://github.com/catchorg/Catch2.git
cd Catch2
git checkout v2.11.1
```

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

```shell
cd /Workspace
git clone https://vcgit.hhi.fraunhofer.de/jct-vc/HM.git HM-16.16
cd HM-16.16
git checkout HM-16.16
```

TMIV includes a build script for HM.

### Instructions to build and install TMIV

To obtain the branch to this document:

```shell
cd /Workspace
git clone http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1.git
cd TM1
git checkout master
```

To obtain the latest release of TMIV instead:

```shell
    cd /Workspace
    git clone https://gitlab.com/mpeg-i-visual/tmiv.git
```

Open the CMake GUI and specify:

* Where the source directory is: /Workspace/TM1
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
HM_BUILD_TAPPDECODER and HM_BUILD_TAPPENCODER are selected, then the
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
1. Run HM encoder on all video sub bitstreams
1. Run TMIV multiplexer to form the output bitstream
1. Run TMIV decoder to decode the bitstream and render a viewport

Use the following steps for uncoded video (i.e. best_reference):

1. Run TMIV encoder
1. Run TMIV decoder to render a viewport

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

## Overview of TMIV encoder parameters

Some of the algorithmic components of the test model have parameters. This
section provides a short description of these parameters in reference to [2] and
the template configuration files. The usage of non-algorithmic parameters such
as filename patterns should be clear from the template configuration files.

### Common encoder parameters

* **blockSize:** int; width and height of the blocks in the block to patch map.
 This is the patch alignment value.
* **geometryScaleEnabledFlag:** bool; when true geometry is downscaled by a
 factor of two in respect to the atlas frame size. Otherwise geometry is at full
 resolution.
* **intraPeriod:** int; the intra patch frame period. This is the step in frame
 order count between consecutive frames that have an atlas tile layer of type
 I_TILE. The test model is not aware of the intra period of the video codec.
 This other intra period is configured independently.
* **maxAtlases:** int; the maximum number of transmitted atlases in total.
* **maxEntities:** int; the maximum number of entities whereby "1" disables
 entity-based coding.
* **maxLumaSamplerate:** float; the maximum number of luma samples per second
 counted over all atlases, groups and components. This parameter communicates
 the equally-named CTC constraint.
* **maxLumaPictureSize:** int; the maximum number of samples per atlas frame,
 which corresponds to the maximum number of samples per texture attribute
 video frame. This parameter communicates the equally-named CTC constraint.
* **numGroups:** int; the number of groups of views (at least 1). The groups are
 formed only when the group-based encoder is selected, but other components use
 this value for instance to calculate suitable atlas frame sizes per group.
* **numberOfFrames:** int; the number of frames to encode.
* **startFrame:** int; skip this many source frames.
* **OmafV1CompatibleFlag:** bool; when enabled the equally-named flag is
 written in the bitstream.
* **explicitOccupancy:** bool; use occupancy video data (OVD) instead of
 depth/occupancy coding within geometry video data (GVD). Make sure to use
 ExplicitOccupancy as the geometry quantizer.

### Geometry quality assessment

* **blendingFactor:** float; for every reprojected pixel it is checked if
reprojected geometry value is higher than 1.0 - _blendingFactor_ of geometry
value of collocated pixel or any of its neighbors in the target view.
* **maxOutlierRatio:** float; pixel outlier threshold above which the geometry
quality is judged to be low.

### Encoder and group-based encoder

Most of the parameters are defined in the root. The exception is:

* **dilate:** int; number of dilation steps on the aggregated pruning mask.
 This parameter is only in effect for low depth quality.

### Geometry quantizer

* **depthOccThresholdIfSet:** int; the value of the depth-occupancy map
 threshold when occupancy information is encoded in the geometry video data of
 a specific view.

### Hierarchical pruner

* **depthParameter:** float; weighting parameter of depth ordering in the trial
 view synthesis that is performed as part of the pruning process.
* **rayAngleParameter:** float; weighting parameter of ray angle in the trial
 view synthesis that is perforemd as part of the pruning process.
* **stretchingParameter:** float; weighting parameter of triangle stretching in
 in the trial view synthesis that is performed as part of the pruning process.
* **maxStretching:** float; the maximum stretching of a trangle in pixel units
 above which the triangle is dropped.
* **dilate:** int; number of dilation steps on the pruning mask.
* **erode:** int; number of erosion steps on the pruning mask.
* **maxDepthError:** float; the maximum relative difference in depth value below which
 the pixel is pruned.
* **maxLumaError:** float; the maximum difference in texture value [0, 1]
 below which the pixel is pruned.
* **maxBasicViewsPerGraph:** int; parameter to control the maximum number of basic
 views per pruning cluster.

### Packer

* **MinPatchSize:** int; is the number of pixels of the smallest border of the
 patch, below which the patch is discarded. Default value is 8.
* **Overlap:** int; is the number of pixels which will be added to a frontier
 of a newly split patch; it prevents seam artefacts. Default value is 1.
* **PiP:** int; is a flag enabling the Patch-in-Patch feature when equal to 1.
 It allows the insertion of patches into other patches. Default value is 1.
* **enableMerging:** bool; enable the patch merging step.

### Basic view allocator

* **maxBasicViewFraction:** float; the maximum number of available luma samples
 that is used for coding views completely.
* **outputAdditionalViews:** bool; when true output basic views and additional
 views. when false output only basic views.
* **minNonCodedViews:** int; the minimum number of source views that will not
 be coded as basic view.

## Overview of TMIV decoder/renderer parameters

### Decoder

* **geometryEdgeMagnitudeTh:** int; parameter of the geometry upscaling, in
 line with the hypothetical reference renderer.
* **maxCurvature:** int;  parameter of the geometry upscaling, in line with the
 hypothetical reference renderer.
* **minForegroundConfidence:** float; parameter of the geometry upscaling, in
 line with the hypothetical reference renderer.

### Renderer

* **angularScaling:** float; Drives the splat size at the warping stage.
* **minimalWeight:** float; Allows for splat degeneracy test at the warping stage.
* **stretchFactor:** float; Limits the splat max size at the warping stage.
* **overloadFactor:** float; Geometry selection parameter at the selection stage.
* **filteringPass:** int; Number of median filtering pass to apply to the visibility map.
* **blendingFactor:** float; Used to control the blending at the shading stage.

## Improving the test model

Core experiments are expected to include the reference software as a subproject
and introduce new components. Alternatively core experiments may branch the test
model. Contributions should be in the form of git pull requests to the
MPEG-internal repository.

## References

* [1] *Call for Proposals on 3DoF+ Visual*, ISO/IEC JTC1/SC29/WG11 MPEG/N18145, Jan. 2019, Marrakesh,
Morocco.

* [2] B. Salahieh, B. Kroon, J. Jung, A. Dziembowski (Eds.), *Test Model 6 for MPEG Immersive Video*,
[ISO/IEC JTC1/SC29/WG11 MPEG/N19483](https://isotc.iso.org/livelink/livelink?func=ll&objId=21345387&objAction=download&viewType=1),
July 2020, Online.

* [3] J. Boyce, R. Doré, V. Kumar Malamal Vadakital (Eds.), *Text of ISO/IEC CD 23090-12 MPEG Immersive Video*,
[ISO/IEC JTC1/SC29/WG11 MPEG/N19482](https://isotc.iso.org/livelink/livelink?func=ll&objId=21346255&objAction=download&viewType=1),
July 2020, Online.

* [4] J. Jung, B. Kroon, J. Boyce, *Common Test Conditions for MPEG Immersive Video*,
[ISO/IEC JTC1/SC29/WG11 MPEG/N19484](https://isotc.iso.org/livelink/livelink?func=ll&objId=21346724&objAction=download&viewType=1),
July 2020, Online.
