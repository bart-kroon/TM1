[![pipeline status](http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1/badges/v7.0-dev/pipeline.svg)](http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1/commits/v7.0-dev)

Test Model for MPEG Immersive Video (TMIV)
==========================================

1. [Introduction](#introduction)
1. [Scope](#scope)
1. [Build and installation instructions](#build-and-installation-instructions)
1. [Instructions to run TMIV](#instructions-to-run-tmiv)
1. [Overview of TMIV encoder parameters](#overview-of-tmiv-encoder-parameters)
1. [Overview of TMIV decoder and renderer parameters](#overview-of-tmiv-decoder-and-renderer-parameters)
1. [Instruction to use TMIV as a library](#instruction-to-use-tmiv-as-a-library)
1. [Structure of the test model](#structure-of-the-test-model)
1. [Contributing to the test model](#contributing-to-the-test-model)
1. [References](#references)

# Introduction

This software project (TMIV-SW) is the ISO/IEC 23090-12 *MPEG Immersive Video* (MIV) reference software and is part of the *Test model for Immersive Video* [[2]](#references).

The MPEG-I project (ISO/IEC 23090) on *Coded Representation of Immersive Media* includes Part 2 *Omnidirectional MediA Format* (OMAF) version 1 published in 2018 that supports 3 Degrees of Freedom (3DoF), where a user’s position is static but its head can yaw, pitch and roll.
However, rendering flat 360° video, i.e. supporting head rotations only, may generate visual discomfort especially when objects close to the viewer are rendered. 6DoF enables translation movements in horizontal, vertical, and depth directions in addition to 3DoF orientations. The
translation support enables interactive motion parallax providing viewers with natural cues to their visual system and resulting in an enhanced perception of volume around them. At the 125th MPEG meeting, a call for proposals [[1]](#references) was issued to enable head-scale movements within a limited space. This has resulted in new MPEG-I Part 12 Immersive Video (MIV).

At the 129th MPEG meeting the fourth working draft of MIV has been realigned to use MPEG-I Part 5 *Video-based Point Cloud Compression* (V-PCC) as a normative reference for terms, defintions, syntax, semantics and decoding processes. At the 130th MPEG meeting this alignment has been completed by restructuring Part 5 in a common specification *Visual Volumetric Video-based Coding* (V3C) and annex H Video-based Point Cloud Compression (V-PCC). V3C provides extension mechanisms for V-PCC and MIV. The terminology in the test model reflects that of V3C and MIV.

# Scope

The normative decoding process for MPEG Immersive Video (MIV) is specified in *Potential improvements of MIV* [[3]](#references). The TMIV reference software provides a reference implementation of non-normative encoding and rendering techniques and the normative decoding process for the MIV
standard.

The test model document [[2]](#references) provides an algorithmic description for the TMIV encoder and decoder/renderer. The purpose of this document is to promote a common understanding of the coding features, in order to facilitate the assessment of the technical impact of new technologies during
the standardization process. *Common Test Conditions for MPEG Immersive Video* [[4]](#references) provides test conditions including TMIV-based anchors. Template configuration files are included with the reference software.

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

1. Visit this URL: https://github.com/catchorg/Catch2/tree/v2.11.1
1. Click on the green button "Code" and select "Download ZIP"
1. Unzip, resulting in a directory `/Workspace/Catch2-2.11.1` such that the file `/Workspace/Catch2-2.11.1/README.md` exists.

### {fmt} string formatting library

1. Visit this URL: https://github.com/fmtlib/fmt/tree/7.0.3
1. Click on the green button "Code" and select "Download ZIP"
1. Unzip, resulting in a directory `/Workspace/fmt-7.0.3` such that the file `/Workspace/fmt-7.0.3/README.md` exists.

### HEVC test model (HM)

1. Visit this URL: https://vcgit.hhi.fraunhofer.de/jct-vc/HM/-/tree/HM-16.16
1. Click on the download button (next to the Clone button) and select "zip"
1. Unzip, resulting in a directory `/Workspace/HM-HM-16.16` such that the file `/Workspace/HM-HM-16.16/README` exists.
1. Rename that directory to `/Workspace/HM-16.16`.

### This project

1. For the latest public release, visit this URL: https://gitlab.com/mpeg-i-visual/tmiv
1. Click on the download button (next to the Clone button) and select "zip"
1. Unzip, resulting in a directory `/Workspace/tmiv-master` such that the file `/Workspace/tmiv-master/README.md` exists.
1. Rename to `/Workspace/tmiv` to match with the following instructions.

To select a specific release instead, e.g. 6.1, use this procedure instead:

1. Visit this URL: https://gitlab.com/mpeg-i-visual/tmiv/-/tree/v6.1 
1. Click on the download button (next to the Clone button) and select "zip"
1. Unzip, resulting in a directory `/Workspace/tmiv-v6.1` such that the file `/Workspace/tmiv-v6.1/README.md` exists.
1. Rename to `/Workspace/tmiv` to match with the following instructions.

The same instruction also works with the internal software repository. Pleae be aware, that the default branch of the internal repository is not `master` but the current development branch, e.g. `v7.0-dev`. This is not a stable version of TMIV.

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

After installation, the TMIV executables `Encoder`, `Decoder` and `Renderer` will be available under the directory `/Workspace/tmiv_install/bin`. By default TMIV only builds the HM modules that are required for TMIV (`TLibCommon` and `TLibDecoder`). When `HM_BUILD_TAPPDECODER` and `HM_BUILD_TAPPENCODER` are selected, then the `TAppDecoder` and `TAppEncoder` tools respectively will also be installed to this directory.

# Instructions to run TMIV

Template configuration files for CTC conditions are available under [ctc_config/](/ctc_config):

* *best_reference* renders from all source views without coding
* *miv_anchor* is the MIV anchor with patches
* *miv_view_anchor* is the MIV view anchor which codes a subset of views completely
* *miv_dsde_anchor* is the MIV decoder-side depth estimating anchor

In addition, there are other configurations available under [test_configs/](/test_configs) to illustrate different aspects of TMIV such as entity-based coding.

The file names of the configuration files, and the file names within them are only examples.

## Instructions for CTC conditions

It is assumed that the reader has read the CTC document [[4]](#references) first. This description does not replace that document.

Use the following steps to encode a bistream and render a viewport:

* MIV anchor and MIV view anchor:
  1. Run the TMIV encoder
  1. Run the HM encoder on all video sub bitstreams
  1. Run the TMIV multiplexer to form the output bitstream
  1. Run the TMIV decoder to decode the bitstream and render a viewport
* MIV decoder-side depth estimating anchor:
  1. Run the TMIV encoder
  1. Run the HM encoder on all video sub bitstreams
  1. Run the TMIV multiplexer to form the output bitstream
  1. Run the TMIV decoder only to decode the bitstream
  1. Run the Immersive Video Depth Estimator (IVDE)
  1. Run the TMIV renderer to render a viewport
* Best reference:
  1. Run the TMIV renderer to render a viewport

## Running the TMIV encoder

For this example, we will be using the MIV anchor [TMIV_A17_SA.json](/ctc_config/miv_anchor/TMIV_A17_SA.json) configuration on the `ClassroomVideo` sequence (SA). This file contains a good choice of parameters, you only need to adapt a few variables:

1. Place the color and depth videos [[4]](#references) in a folder. Make sure to comply to the naming scheme defined in `SourceGeometryPathFmt` and `SourceTexturePathFmt`. Your organization or one of the maintainers of this repository may be able to provide the test sequences to you.

1. The files' naming scheme can for example be `{}_depth_{}x{}_yuv420p16le.yuv`. The curly braces are placeholders for (in sequence)
    1. camera name, as defined in the sequence-specific configuration files, e.g. v13
    1. Horizontal resolution of the video
    1. Vertical resolution of the video

    such that a texture video file from camera `v0` with resolution 4096x2048 pixels should be named `v0_texture_4096x2048_yuv420p10le.yuv`.
1. Set `SourceGeometryBitDepth` to the bits per color channel in your source geometry file.
1. Point to the video file directory by providing the path to configuration variable `SourceDirectory`.
1. You may set `OutputDirectory` to a custom existing directory.

Finally, assuming that you have built and installed the encoder application, you can start it from the command line:

```shell
/Workspace/tmiv_install/bin/Encoder -c /Workspace/tmiv/ctc_config/miv_anchor/TMIV_A17_SA.json
```

This will result in the following files in the `OutputDirectory`:

* One `.bit` file e.g. `ATL_SA.bit`, as defined by `BitstreamPath`, containing metadata and patch data.
* YUV files for each component of each atlas. Components may be any of texture, geometry or occupancy corresponding to the `haveTextureVideo`, `haveGeometryVideo` and `haveOccupancyVideo` TMIV encoder parameters.

## Running the HM encoder

After TMIV encoding, run HM on **all** resulting YUV files. If you have configured the project with `BUILD_HM=ON, BUILD_TAppEncoder=TRUE, and BUILD_TAppDecoder=TRUE`, then the HM executables are available in the TMIV installation directory. To encode one YUV sequence, run e.g.

```shell
/Workspace/tmiv_install/bin/TAppEncoder \
  -c /Workspace/tmiv/ctc_config/miv_anchor/encoder_randomaccess_main10.cfg \
  -c /Workspace/tmiv/ctc_config/miv_anchor/HM_A17_TT_SA.cfg \
  -f 100 -wdt 2320 -hgt 960 -i TG_00_960x2320_yuv420p10le.yuv -b tg_01.bin
```

Whereby `\` is used to indicate line breaks in this manual.

The order of config files for HM is important! Later ones overwrite earlier ones, command line parameters overwrite config files.

### Running the TMIV multiplexer

To run the TMIV multiplexer, set everything in the multiplexer configuration file, for an example see [ctc_config/miv_anchor/Mux_A17_SA.json](/ctc_config/miv_anchor/Mux_A17_SA.json). Careful: there is no input folder for this. Make sure that variables `AttributeVideoDataSubBitstreamPathFmt` and `GeometryVideoDataSubBitstreamPathFmt` are either correct relative paths to you call location, or the correct absolute paths. Then, run

```shell
/Workspace/tmiv_install/bin/Multiplexer -c /Workspace/ctc_config/miv_anchor/Mux_A17.json
```

## Running the TMIV decoder

You may choose to render to either a source view (e.g. `v0`) for objective evaluation, or render according to a pose trace (e.g. `p01`). Pose traces are available under [ctc_config/pose_traces](/ctc_config/pose_traces). For clarity two (almost identical) configuration files are provided for each of the test conditions, for example:

* [ctc_config/miv_anchor/TMIV_A17_SA_v0.json](ctc_config/miv_anchor/TMIV_A17_SA_v0.json)
* [ctc_config/miv_anchor/TMIV_A17_SA_p01.json](ctc_config/miv_anchor/TMIV_A17_SA_p01.json)

Note that it is not needed to decode video with HM because the HM decoder is integrated into the TMIV decoder. The input of the decoder is a single MIV bitstream including HEVC sub-bitstreams.

It is possible to use YUV video input, for instance to support experiments with alternative video codecs such as VTM, but this is **advanced use** and **not recommended** in general. To enable decoding of MIV bitstreams with out-of-band decoded video sub-bitstreams, add the  `OccupancyVideoDataPathFmt`,`GeometryVideoDataPathFmt` and/or `AttributeVideoDataPathFmt` to the configuration file. The path formats match those of the encoder configuration, see for instance [TMIV_A17_SA.json](/ctc_config/miv_anchor/TMIV_A17_SA.json). When the decoder detects that a video sub-bitstream is not present in the MIV bitstream, it will use such a parameter to calculate the path to a YUV file and load frames from that. The format and resolution of the YUV file is dictated by the MIV bitstream.

## Running the TMIV renderer

The TMIV renderer was added to support the MIV decoder-side depth estimating anchor. The application has similar input to the TMIV encoder (input views and camera parameters) and similar output to the TMIV decoder (rendered viewport).

As with the TMIV decoder you may choose to render to either a source view (e.g. `v0`) for objective evaluation, or render according to a pose trace (e.g. `p01`). A suitable configuration file to try out the TMIV renderer is the one of the best reference condition:

* [ctc_config/best_reference/TMIV_render_R17_SA_p01.json](ctc_config/best_reference/TMIV_render_R17_SA_p01.json)

# Overview of TMIV encoder parameters

Some of the algorithmic components of the test model have parameters. This section provides a short description of these parameters in reference to [[2]](#references) and the template configuration files. The usage of non-algorithmic parameters such as filename patterns should be clear from the template configuration files.

### Common encoder parameters

* Input frame selection:
  * **numberOfFrames:** int; the number of frames to encode.
  * **startFrame:** int; skip this many source frames.
* Output video sub-bitstreams:
  * **haveOccupancyVideo:** bool; output occupancy video data (OVD) instead of  depth/occupancy coding within geometry video data (GVD). Make sure to use ExplicitOccupancy as the geometry quantizer.
  * **haveTextureVideo:** bool; output attribute video data (AVD) to encode the texture attribute. When false texture data is still needed as input of the test model.
  * **haveGeometryVideo:** bool; output geometry video data (GVD) to encode depth and optionally also occuapncy information. Without geometry, depth estimation is shifted from a pre-encoding to a post-decoding process.
  * **geometryScaleEnabledFlag:** bool; when true geometry is downscaled by a factor of two in respect to the atlas frame size. Otherwise geometry is at full resolution.
* Atlas frame size calculation and packing:
  * **intraPeriod:** int; the intra patch frame period. This is the step in frame order count between consecutive frames that have an atlas tile layer of type I_TILE. The test model is not aware of the intra period of the video codec. This other intra period is configured independently.
  * **maxAtlases:** int; the maximum number of transmitted atlases in total.
  * **maxEntities:** int; the maximum number of entities whereby "1" disables entity-based coding.
  * **maxLumaSamplerate:** float; the maximum number of luma samples per second counted over all atlases, groups and components. This parameter communicates the equally-named CTC constraint.
  * **maxLumaPictureSize:** int; the maximum number of samples per atlas frame, which corresponds to the maximum number of samples per texture attribute video frame. This parameter communicates the equally-named CTC constraint.
  * **numGroups:** int; the number of groups of views (at least 1). The groups are formed only when the group-based encoder is selected, but other components use this value for instance to calculate suitable atlas frame sizes per group.
* Metadata:
  * **OmafV1CompatibleFlag:** bool; when enabled the equally-named flag is written in the bitstream.

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

# Overview of TMIV decoder and renderer parameters

## Decoder

* **geometryEdgeMagnitudeTh:** int; parameter of the geometry upscaling, in
 line with the hypothetical reference renderer.
* **maxCurvature:** int;  parameter of the geometry upscaling, in line with the
 hypothetical reference renderer.
* **minForegroundConfidence:** float; parameter of the geometry upscaling, in
 line with the hypothetical reference renderer.

## Renderer

* **angularScaling:** float; Drives the splat size at the warping stage.
* **minimalWeight:** float; Allows for splat degeneracy test at the warping stage.
* **stretchFactor:** float; Limits the splat max size at the warping stage.
* **overloadFactor:** float; Geometry selection parameter at the selection stage.
* **filteringPass:** int; Number of median filtering pass to apply to the visibility map.
* **blendingFactor:** float; Used to control the blending at the shading stage.

# Instruction to use TMIV as a library

In the `CMakeLists.txt` of the larger project, use the line:

```CMake
find_package(TMIV 7.0.0 REQUIRED)
```

The `TMIV_DIR` variable needs to be set to the directory named `/Workspace/tmiv_install/lib/cmake/TMIV` or `/Workspace/tmiv_install/lib64/cmake/TMIV` depending on the platform. 

After that the targets `TMIV::DecoderLib`, `TMIV::EncoderLib`, etc. are available as target dependencies.

# Structure of the test model

This software consists of multiple executables and static libraries. Below figure is the CMake module dependency graph of TMIV 5.0. The most important executables are Encoder and Decoder. When enabled, the project also includes the HM executables TAppEncoder and TAppDecoder.

![CMake module graph](doc/module_graph.png)

Each module consists of one or more components. Most components derive from an interface and some interfaces have multiple alternative components. For instance, the ViewOptimizerLib includes an IViewOptimizer interface with NoViewOptimizer and ViewReducer components that implement that interface.

# Contributing to the test model

Core experiments are expected to include the reference software as a subproject
and introduce new components. Alternatively core experiments may branch the test
model. Contributions should be in the form of git merge requests to the
MPEG-internal repository. See [CONTRIBUING.md](CONTRIBUTING.md) for further info.

# References

* [1] *Call for Proposals on 3DoF+ Visual*, ISO/IEC JTC 1/SC 29/WG 11 N 18145, Jan. 2019, Marrakesh, Morocco.
* [2] B. Salahieh, B. Kroon, J. Jung, A. Dziembowski (Eds.), *Test Model 7 for MPEG Immersive Video*, [ISO/IEC JTC 1/SC 29/WG 04 N 0005](https://www.mpegstandards.org/wp-content/uploads/mpeg_meetings/132_OnLine/w19678.zip), October 2020, Online.
* [3] J. Boyce, R. Doré, V. Kumar Malamal Vadakital (Eds.), *Potential Improvements of MPEG Immersive Video*, [ISO/IEC JTC 1/SC 29/WG 04 N 0004](https://www.mpegstandards.org/wp-content/uploads/mpeg_meetings/132_OnLine/w19677.zip), October 2020, Online.
* [4] J. Jung, B. Kroon, J. Boyce (Eds.), *Common Test Conditions for MPEG Immersive Video*, [ISO/IEC JTC 1/SC 29/WG 04 N 0006](https://www.mpegstandards.org/wp-content/uploads/mpeg_meetings/132_OnLine/w19679.zip), October 2020, Online.
