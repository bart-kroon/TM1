# Instructions to run TMIV

Configuration files for CTC conditions are available under [config/ctc/](/config/ctc):

* *best_reference* renders from all source views without coding
* *miv_anchor* is the MIV anchor with patches
* *miv_view_anchor* is the MIV view anchor which codes a subset of views completely
* *miv_dsde_anchor* is the MIV decoder-side depth estimating anchor

In addition, there are other configurations available under [config/test/](/config/test) to illustrate different aspects of TMIV such as entity-based coding or multi-plane image (MPI) coding.

The file names of the configuration files, and the file names within them are only examples.

## Instructions for CTC conditions

It is assumed that the reader has read the CTC document [[5]](/README.md#references) first. This description does not replace that document.

Use the following steps to encode a bitstream and render a viewport:

* MIV anchor and MIV view anchor:
    1. Run the TMIV encoder
    1. Run the VVenC encoder on all video sub bitstreams
    1. Run the VVdeC decoder on all video sub bitstreams
    1. Run the TMIV decoder to decode the MIV bitstream to render a viewport
* MIV decoder-side depth estimating anchor:
    1. Run the TMIV encoder
    1. Run the VVenC encoder on all video sub bitstreams
    1. Run the VVdeC decoder on all video sub bitstreams
    1. Run the TMIV decoder only to decode the bitstream
    1. Run the Immersive Video Depth Estimator (IVDE)
    1. Run the TMIV renderer to render a viewport
* Best reference:
    1. Run the TMIV renderer to render a viewport

### Running the TMIV encoder

For this example, we will be using the MIV anchor [A_1_TMIV_encode.json](/config/ctc/miv_anchor/A_1_TMIV_encode.json) configuration and [A.json](/config/ctc/sequences/A.json) sequence configuration on sequence A (ClassroomVideo) with 97 input frames starting from frame 23.

1. Place the color and depth videos [[5]](/README.md#references) in a folder arbitrarily named `/Content` in this description.
    * Your organization or one of the maintainers of this repository may be able to provide the test sequences to you.
    * Make sure to comply to the naming scheme defined in `inputTexturePathFmt` and `inputGeometryPathFmt`.
    * For example, when `inputDirectory` is equal to `/Content`,
    * and given that `inputGeometryPathFmt` is equal to `{1}/{3}_depth_{4}x{5}_{6}.yuv`,
    * then for content ID `A` (ClassroomVideo),
    * a source view `v3` from the set of source views,
    * a video format derived from the sequence configuration `yuv420p16le`,
    * the calculated path for the uncompressed depth map sequene of that view is `/Content/A/v3_depth_4096x2048_yuv420p16le.yuv`.
1. Choose an output directory, arbitrarily called `/Experiment` in this description. The directory will be created when it does not yet exist.
1. Finally, assuming that you have built and installed the encoder application, you can start it from the command line:

```shell
/Workspace/tmiv_install/bin/Encoder -n 97 -s A -f 23 \
    -c /Workspace/tmiv/config/ctc/miv_anchor/A_1_TMIV_encode.json \
    -p configDirectory /Workspace/tmiv/config \
    -p inputDirectory /Content \
    -p outputDirectory /Experiment
```

Whereby `\` is used to indicate line breaks in this manual.

When the same parameter is provided multiple times on the command-line, through `-c` or `-p`, then the right-most argument has precedence.

This will in general result in the following files under the `outputDirectory`:

* A bitstream with the path based on `outputBitstreamPathFmt`, containing metadata and patch data.
* YUV files for each component of each atlas with paths based on `outputGeometryVideoDataPathFmt`, `outputOccupancyVideoDataPathFmt`, `outputTransparencyVideoDataPathFmt` and/or `outputTextureVideoDataPathFmt`. Components may be any of texture, transparency, geometry or occupancy corresponding to the `haveTextureVideo`, `haveTransparencyVideo` :construction:, `haveGeometryVideo` and `haveOccupancyVideo` TMIV encoder parameters.

In this example the following files will be produced:

```
/Experiment/A97/A/TMIV_A97_A.bit
/Experiment/A97/A/TMIV_A97_A_geo_c00_2048x1088_yuv420p10le.yuv
/Experiment/A97/A/TMIV_A97_A_geo_c01_2048x1088_yuv420p10le.yuv
/Experiment/A97/A/TMIV_A97_A_tex_c00_4096x2176_yuv420p10le.yuv
/Experiment/A97/A/TMIV_A97_A_tex_c01_4096x2176_yuv420p10le.yuv
```

### Running the VVenC encoder

As TMIV is agnostic to the 2D video codec, you can use a codec of your choice in out-of-band video coding.
The CTC provide configurations VVenC and VVdeC, so we describe their usage here.
You can alternatively use HEVC (HM) for encoding and decoding.

After TMIV encoding, run VVenC on **all** resulting YUV files.
Per default, executable `vvencFFapp` is available in the TMIV installation directory.
To encode one YUV sequence, use the following settings to run VVenC:

1. Use the supplied configuration file with flag `-c` (there will be a configuration for each component, in this case geometry and texture)
1. Specify the number of input frames (`-f` parameter)
1. Derive the frame width and height from the paths of the YUV files (or encoder log), pass them as `-s WIDTHxHEIGHT`
1. Derive the frame rate (`-fr` parameter) from the sequence configuration (often 30 or 25)
1. Choose a QP (`-q` parameter) or look up in the CTC document. Note that the QPs in the CTC may vary per video component.

For example:

```shell
/Workspace/tmiv_install/bin/vvencFFapp \
  -c /Workspace/tmiv/config/ctc/miv_anchor/A_2_VVenC_encode_tex.cfg \
  -i /Experiment/A97/A/TMIV_A97_A_tex_c00_4096x2176_yuv420p10le.yuv \
  -b /Experiment/A97/A/QP3/TMIV_A97_A_QP3_tex_c00.bit \
  -s 4096x2176 -q 30 -f 97 -fr 30
```

In this example the following files will be produced after four invocations:

```
/Experiment/A97/A/QP3/TMIV_A97_A_QP3_geo_c00.bit
/Experiment/A97/A/QP3/TMIV_A97_A_QP3_geo_c01.bit
/Experiment/A97/A/QP3/TMIV_A97_A_QP3_tex_c00.bit
/Experiment/A97/A/QP3/TMIV_A97_A_QP3_tex_c01.bit
```

### Running the VVdeC decoder

After VVenC encoding, run VVdeC on **all** resulting YUV files.
Per default, executable `vvdecapp` is available in the TMIV installation directory.
To decode one YUV sequence, use the following settings to run VVdeC:

1. Specify the input VVC bitstream with parameter `-b`
1. Specify the output yuv file with parameter `-o`

For example:

```shell
/Workspace/tmiv_install/bin/vvdecapp \
  -b /Experiment/A97/A/QP3/TMIV_A97_A_QP3_tex_c00.bit \
  -o /Experiment/A97/A/QP3/TMIV_A97_A_QP3_tex_c00_4096x2176_yuv420p10le.yuv
```

In this example the following files will be produced after four invocations:

```
/Experiment/A97/A/QP3/TMIV_A97_A_QP3_geo_c00_4096x2176_yuv420p10le.yuv
/Experiment/A97/A/QP3/TMIV_A97_A_QP3_geo_c01_4096x2176_yuv420p10le.yuv
/Experiment/A97/A/QP3/TMIV_A97_A_QP3_tex_c00_4096x2176_yuv420p10le.yuv
/Experiment/A97/A/QP3/TMIV_A97_A_QP3_tex_c01_4096x2176_yuv420p10le.yuv
```

### TMIV decoding (with out-of-band video)

For out-of-band decoding, the TMIV decoder requires the TMIV encoded bitstream and the decoded YUV files.
Invoke the decoder with the following arguments:

1. Choose as input directory the output directory of the TMIV encoding step, that is `/Experiment` in this description.
1. Choose as `inputBitstreamPathFmt` a path representing your TMIV bitstream
1. Choose as `inputGeometryVideoFramePathFmt` a path representing the geometry bitstreams
1. Choose as `inputTextureVideoFramePathFmt` a path representing the texture bitstreams
1. Choose an output directory, in this example again `/Experiment` is used to have the viewport videos next to the VVC bitstreams.
1. Define render targets, if any. (The decoder can also produce other outputs such as multiview reconstruction or block to patch maps.)
    * Do not define any render targets for the MIV DSDE anchor.
    * The view name (`-v` argument) may be used multiple times to reconstruct source views and interpolate intermediate views.
    * The pose trace name (`-P` argument) may be used multiple times to render pose trace videos.
    * Rendering tasks are run _sequentially_.
      It is advised to run multiple processes in parallel to speed up anchor generation.
1. Specify the number of output frames (`-N` argument) e.g. 300:
    * For views the actual number of output frames is never more than the number of input frames,
    * For pose traces the sequence of decoded input frames is looped (by mirroring; repeatedly traversing the sequence forth and back) and the number of output frames can exceed the number of input frames.
1. Finally, assuming that you have built and installed the decoder application, you can start it from the command line:

```shell
/Workspace/tmiv_install/bin/Decoder -n 97 -N 300 -s A -r QP3 -v v11 -P p02 \
    -c /Workspace/tmiv/config/ctc/miv_anchor/A_4_TMIV_decode.json \
    -p configDirectory /Workspace/tmiv/config \
    -p inputDirectory /Experiment \
    -p outputDirectory /Experiment
```

When the same parameter is provided multiple times on the command-line, through `-c` or `-p`, then the right-most argument has precedence.

This will in general result in the following files under the `outputDirectory`:

* Block to patch maps with the path based on `outputBlockToPatchMapPathFmt`.
* Reconstructed multiview video based on `outputMultiviewGeometryPathFmt`, `outputOccupancyVideoDataPathFmt`, `outputMultiviewTransparencyPathFmt` and/or `outputMultiviewTexturePathFmt`.
* Rendered viewport videos based on `outputViewportGeometryPathFmt` and/or `outputViewportTexturePathFmt`.
* Reconstructed sequence configuration for each frame at which it changes based on `outputSequenceConfigPathFmt`.

In this example the following files will be produced:

```
/Experiment/A97/A/QP3/A97_A_QP3_p02_tex_2048x2048_yuv420p10le.yuv
/Experiment/A97/A/QP3/A97_A_QP3_v11_tex_4096x2048_yuv420p10le.yuv
```

### Running the TMIV renderer

The TMIV renderer was added to support the MIV decoder-side depth estimating anchor. The application has similar input to the TMIV encoder (input views and camera parameters) and similar output to the TMIV decoder (rendered viewport).

For this example, we will be using the best reference [R_1_TMIV_render.json](/config/ctc/best_reference/R_1_TMIV_render.json) configuration and [A.json](/config/ctc/sequences/A.json) sequence configuration.

1. Place the color and depth videos [[5]](/README.md#references) in a folder arbitrarily named `/Content` in this description.
1. Choose an output directory, arbitrarily called `/Experiment` in this description. The directory will be created when it does not yet exist.
1. Define render targets (having none is allowed but not that useful):
    * The view name (`-v` argument) may be used multiple times to reconstruct source views and interpolate intermediate views.
    * The pose trace name (`-P` argument) may be used multiple times to render pose trace videos.
    * Rendering tasks are run _sequentially_. It is adviced to run multiple processes in parallel to speed up anchor generation.
1. Specify the number of input frames (`-n` argument), e.g. 97.
1. Specify the number of output frames (`-N` argument), e.g. 300:
    * For views the actual number of output frames is never more than the number of input frames,
    * For pose traces the input frames are mirrored and the number of output frames can exceed the number of input frames.
1. Specify the start frame (`-f` argument), e.g. 23.
1. Finally, assuming that you have built and installed the renderer application, you can start it from the command line:

```shell
/Workspace/tmiv_install/bin/Renderer -n 97 -N 300 -s A -r R0 -v v11 -P p02 -f 23 \
    -c /Workspace/tmiv/config/ctc/best_reference/R_1_TMIV_render.json \
    -p configDirectory /Workspace/tmiv/config \
    -p inputDirectory /Content \
    -p outputDirectory /Experiment
```

When the same parameter is provided multiple times on the command-line, through `-c` or `-p`, then the right-most argument has precedence.

This will in general result in the following files under the `outputDirectory`:

* Rendered viewport videos based on `outputViewportGeometryPathFmt` and/or `outputViewportTexturePathFmt`.

In this example the following files will be produced:

```
/Experiment/R97/A/R0/R97_A_R0_p02_tex_2048x2048_yuv420p10le.yuv
/Experiment/R97/A/R0/R97_A_R0_v11_tex_4096x2048_yuv420p10le.yuv
```

Note that when running the TMIV renderer to generate the MIV decoder-side depth estimating anchor, the command line to use for reconstructing a source view should be:

```shell
/Workspace/tmiv_install/bin/Renderer -n 17 -N 17 -s A -r QP1 -v v11 \
    -c /Workspace/tmiv/config/ctc/miv_dsde_anchor/G_6_TMIV_render.json \
    -p configDirectory /Workspace/tmiv/config \
    -p inputDirectory /Experiment \
    -p outputDirectory /Experiment
```

This will in general result in the following files under the `outputDirectory`:

```
/Experiment/G17/A/QP1/G17_A_QP1_v11_tex_4096x2048_yuv420p10le.yuv
```

## Instructions for in-band HEVC video coding with HM

The test model has the capability of muxing, demuxing and in-memory decoding of HEVC video using the HEVC Test Model (HM).

### Running the HM encoder

After TMIV encoding (analogous as TMIV encoding for VVC in the CTC), only with the config in `tmiv/config/test/miv_multiplex/V_1_TMIV_encode.json`, run HM on **all** resulting YUV files.
To encode one YUV sequence, use the following settings to run HM:

1. Use the supplied configuration file (there will be a configuration for each component, in this case geometry and texture)
1. Specify the number of input frames (`-f` parameter)
1. Derive the frame width and height from the paths of the YUV files (or encoder log)
1. Derive the frame rate (`-fr` parameter) from the sequence configuration (often 30 or 25)
1. Choose a QP (`-q` parameter) or look up in the CTC document. Note that the QPs in the CTC may vary per video component.

For example:

```shell
/Workspace/tmiv_install/bin/TAppEncoder \
  -c /Workspace/tmiv/config/test/miv_multiplex/V_2_HM_encode_tex.cfg \
  -i /Experiment/V97/A/TMIV_V97_A_tex_c00_4096x2176_yuv420p10le.yuv \
  -b /Experiment/V97/A/QP3/TMIV_V97_A_QP3_tex_c00.bit \
  -wdt 2320 -hgt 960 -q 30 -f 97 -fr 30
```

In this example the following files will be produced after four invocations:

```
/Experiment/V97/A/QP3/TMIV_V97_A_QP3_geo_c00.bit
/Experiment/V97/A/QP3/TMIV_V97_A_QP3_geo_c01.bit
/Experiment/V97/A/QP3/TMIV_V97_A_QP3_tex_c00.bit
/Experiment/V97/A/QP3/TMIV_V97_A_QP3_tex_c01.bit
```

### Running the TMIV multiplexer

1. Choose as input directory the output directory of the HM step, that is `/Experiment` in this description.
1. Choose an output directory, in this example `/Experiment` is used to have the multiplexed bitstream next to the HM bitstreams.
1. The test ID (`-r` argument) is used to tag multiple video encodings at different settings, e.g. QP1, QP2, etc. or R0 for lossless.
1. Finally, assuming that you have built and installed the multiplexer application, you can start it from the command line:

```shell
/Workspace/tmiv_install/bin/Multiplexer -n 97 -s A -r QP3 \
    -c /Workspace/tmiv/config/test/miv_in-band_hm/H_3_TMIV_mux.json \
    -p configDirectory /Workspace/tmiv/config \
    -p inputDirectory /Experiment \
    -p outputDirectory /Experiment
```

When the same parameter is provided multiple times on the command-line, through `-c` or `-p`, then the right-most argument has precedence.

This will in general result in the following file under the `outputDirectory`:

* A bitstream with the path based on `outputBitstreamPathFmt`, containing metadata, patch data and video sub bitstreams.

In this example the following file will be produced:

```
/Experiment/V97/A/QP3/TMIV_V97_A_QP3.bit
```

### Running the TMIV decoder

1. Choose as input directory the output directory of the multiplexing step, that is `/Experiment` in this description.
1. Choose an output directory, in this example again `/Experiment` is used to have the viewport videos next to the HM bitstreams.
1. Define render targets if any. (The decoder can also produce other outputs such as multiview reconstruction or block to patch maps.)
    * The view name (`-v` argument) may be used multiple times to reconstruct source views and interpolate intermediate views.
    * The pose trace name (`-P` argument) may be used multiple times to render pose trace videos.
    * Rendering tasks are run _sequentially_. It is advised to run multiple processes in parallel to speed up anchor generation.
1. Specify the number of output frames (`-N` argument) e.g. 300:
    * For views the actual number of output frames is never more than the number of input frames,
    * For pose traces the sequence of decoded input frames is looped (by mirroring; repeatedly traversing the sequence forth and back) and the number of output frames can exceed the number of input frames.
1. Finally, assuming that you have built and installed the decoder application, you can start it from the command line:

```shell
/Workspace/tmiv_install/bin/Decoder -n 97 -N 300 -s A -r QP3 -v v11 -P p02 \
    -c /Workspace/tmiv/config/test/miv_in-band_hm/H_4_TMIV_decode.json \
    -p configDirectory /Workspace/tmiv/config \
    -p inputDirectory /Experiment \
    -p outputDirectory /Experiment
```

When the same parameter is provided multiple times on the command-line, through `-c` or `-p`, then the right-most argument has precedence.

This will in general result in the following files under the `outputDirectory`:

* Block to patch maps with the path based on `outputBlockToPatchMapPathFmt`.
* Reconstructed multiview video based on `outputMultiviewGeometryPathFmt`, `outputOccupancyVideoDataPathFmt`, `outputMultiviewTransparencyPathFmt` and/or `outputMultiviewTexturePathFmt`.
* Rendered viewport videos based on `outputViewportGeometryPathFmt` and/or `outputViewportTexturePathFmt`.
* Reconstructed sequence configuration for each frame at which it changes based on `outputSequenceConfigPathFmt`.

In this example the following files will be produced:

```
/Experiment/V97/A/QP3/V97_A_QP3_p02_tex_2048x2048_yuv420p10le.yuv
/Experiment/V97/A/QP3/V97_A_QP3_v11_tex_4096x2048_yuv420p10le.yuv
```

## Instructions for multiplane image coding

The multi-plane image (MPI) encoder is able to encode an MPI sequence consisting of texture and transparency components.
The input is a succession of very sparse layers of texture and transparency.
In the example given in [M.json](/config/ctc/sequences/M.json), there are 423 such layers.

To limit memory consumption and processing time, the Packed Compressed Storage (PCS) format is the only MPI input format for TMIV encoder (from TMIV 8.0).
In PCS format, texture and transparency are stored together in the same `.pcs` file, frame by frame.
This format is explained in detail the Test Model document.

Conversion from PCS format to raw storage format is possible inside TMIV (see [Running the MPI converter](#running-the-mpi-converter))

A specific MPI encoder is to be used to deal with this type of content, and a specific MPI synthesizer is needed at rendering stage.
But actually, even if the internal processing is different, the exact same steps as for MIV anchor can be used to encode a bitsream for MPI content and render a viewport:
In the example MPI configuration, HM is used in combination with the TMIV multiplexer, to create a single bitstream.

* MPI test:
    1. Run the TMIV encoder with MPI encoder (see [Running the TMIV encoder with MPI](#running-the-tmiv-encoder-with-mpi))
    1. Run the HM encoder on all video sub bitstreams (see [Running the HM encoder with MPI](#running-the-hm-encoder-with-mpi))
    1. Run the TMIV multiplexer to form the output bitstream (see [Running the TMIV multiplexer with MPI](#running-the-tmiv-multiplexer-with-mpi))
    1. Run the TMIV decoder with MPI synthesizer to decode the bitstream and render a viewport (see [Running the TMIV decoder with MPI](#running-the-tmiv-decoder-with-mpi))

The only difference is in the input, we do only use `inputMpiPcsPathFmt`.
And also some specific parameters for [MPI encoder](/doc/configuring.md#mpi-encoder) and [MPI synthesizer](/doc/configuring.md#mpi-synthesizer).


#### Running the TMIV encoder with MPI
For this example, we will be using the MIV anchor [M_1_TMIV_encode.json](/config/test/miv_mpi/M_1_TMIV_encode.json) configuration and [M.json](/config/ctc/sequences/M.json) sequence configuration.

1. Place the `.pcs` file in a folder arbitrarily named `/Content` in this description.
    * Your organization or one of the maintainers of this repository may be able to provide the test sequences to you.
    * Make sure to comply to the naming scheme defined in `inputMpiPcsPathFmt`.
    * For example, when `inputDirectory` is equal to `/Content`,
    * and given that `inputMpiPcsPathFmt` is equal to `{1}/{3}.pcs`,
    * then for content ID `M` (Mpi_Fan),
    * and source view `mpi`,
    * the calculated path for the `.pcs` file of that view is `/Content/M/mpi.pcs`.
1. Choose an output directory, arbitrarily called `/Experiment` in this description. The directory will be created when it does not yet exist.
1. Finally, assuming that you have built and installed the TMIV MPI encoder application, you can start it from the command line:

```shell
/Workspace/tmiv_install/bin/MpiEncoder -n 17 -s M \
    -c /Workspace/tmiv/config/test/miv_mpi/M_1_TMIV_encode.json \
    -p configDirectory /Workspace/tmiv/config \
    -p inputDirectory /Content \
    -p outputDirectory /Experiment
```

Following generated atlas for the example will be produced:
```
/Experiment/M17/M/TMIV_M17_M.bit
/Experiment/M17/M/TMIV_M17_M_tra_c00_4096x4096_yuv420p10le.yuv
/Experiment/M17/M/TMIV_M17_M_tex_c00_4096x4096_yuv420p10le.yuv
```
#### Running the HM encoder with MPI
This section is similar to [Running the HM encoder](#running-the-hm-encoder).
Use `test/miv_mpi/M_2_HM_encode_tra.cfg` to encode transparency atlas, and `test/miv_mpi/M_2_HM_encode_tex.cfg` to encode texture atlas.
Following HM encoded files at QP3 for the example will be produced:
```
/Experiment/M17/M/QP3/TMIV_M17_M_QP3_tra_c00.bit
/Experiment/M17/M/QP3/TMIV_M17_M_QP3_tex_c00.bit
```

#### Running the TMIV multiplexer with MPI
This section is similar to [Running the TMIV multiplexer](#running-the-tmiv-multiplexer).
Use `test/miv_mpi/M_3_TMIV_mux.json` to generate the multiplexed stream.
Following multiplexed file for the example will be produced:
```
/Experiment/M17/M/QP3/TMIV_M17_M_QP3.bit
```

#### Running the TMIV decoder with MPI
This section is similar to [Running the TMIV decoder](#running-the-tmiv-decoder).
Use `test/miv_mpi/M_4_TMIV_decode.json` to run the TMIV decoder with MPI synthesizer.
The following command:
```shell
/Workspace/tmiv_install/bin/Decoder -n 17 -N 300 -s M -r QP3 -P p01 \
    -c /Workspace/tmiv/config/test/miv_mpi/M_4_TMIV_decode.json /
    -p configDirectory /Workspace/tmiv/config /
    -p inputDirectory /Experiment /
    -p outputDirectory /Experiment
```
will generate the following synthesized pose trace:
```
/Experiment/M17/M/QP3/M17_M_QP3_p01_tex_1920x1080_yuv420p10le.yuv
```

#### Running the MPI converter
##### PCS to raw conversion
An MPI sequence in PCS format can be converted to raw storage to allow visualization.

Input path format `inputMpiPcsPathFmt` and output path formats `outputTexturePathFmt` and `outputTransparencyPathFmt` are used for this conversion.

Assuming that the .pcs file of the MPI content is available in the `/Content` directory:
```
/Content/M/mpi.pcs
```
running the following command:
```shell
/Workspace/tmiv_install/bin/MpiPcs -n 17 -f 0 -s M -x pcs2raw \
    -c /Workspace/tmiv/config/test/miv_mpi/M_5_MPI_transcode.json /
    -p configDirectory /Workspace/tmiv/config /
    -p inputDirectory /Content /
    -p outputDirectory /Experiment
```
will generate the following texture and transparency files:
```
/Experiment/M/mpi_texture_4176x2024_yuv420p10le.yuv
/Experiment/M/mpi_transparency_4176x2024_yuv420p.yuv
```
##### Raw to PCS conversion
Conversely, an MPI sequence in raw storage format can be converted to PCS format to be input by the TMIV encoder.

Input path formats `inputTexturePathFmt` and `inputTransparencyPathFmt` and output path format `outputMpiPcsPathFmt` are used for this conversion.

Assuming that the texture and transparency files of the MPI content are available in the `/Experiment` directory:
```
/Experiment/M/mpi_texture_4176x2024_yuv420p10le.yuv
/Experiment/M/mpi_transparency_4176x2024_yuv420p.yuv
```
running the following command:
```shell
/Workspace/tmiv_install/bin/MpiPcs -n 17 -f 0 -s M -x raw2pcs \
    -c /Workspace/tmiv/config/test/miv_mpi/M_5_MPI_transcode.json /
    -p configDirectory /Workspace/tmiv/config /
    -p inputDirectory /Experiment /
    -p outputDirectory /Experiment
```
will generate the `.pcs` file:
```
/Experiment/M/mpi.pcs
```

## Instructions for multiplexing externally packed video

### General

TMIV multiplexer provides functionality that allows (re)writing an existing V3C bitstream where each video component is provided separately to a new V3C bitstream that contains a packed video component.
In such a case, the multiplexer re-writes a V3C parameter set and includes packing information based on the provided input and replaces separate video components by one packed video component that was created outside of the TMIV pipeline.

One example of how this TMIV multiplexer can be utilized is when texture and geometry atlases were initially encoded by the Versatile Video Coding (VVC) codec, as independently decodable subpictures and packed by an external tool into one packed video component at a later stage.

The following sections provide an example procedure on how the multiplexing of externally packed video can be performed.
More details about the example with the test results can be found in document m55818.

### TMIV encoding

The atlases (un-encoded video components) are generated with the TMIV encoding procedure e.g. according to the [CTC conditions](http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1/-/blob/main/doc/running.md#running-the-tmiv-encoder).

### VVC encoding

The un-encoded video components are then coded by a VVC codec, e.g. [VVC Test Model (VTM)](https://vcgit.hhi.fraunhofer.de/jvet/VVCSoftware_VTM), with additional constraints.
The dimensions of a subpicture must be a multiple of the CTUs size.
Therefore, the atlases should be padded along the width and height of the atlas, if required, before encoding.
When padding the video components, the information about the original YUV dimensions, and the dimensions after padding should be stored, so they can later be used during the multiplexing process when packing information is created.

In order to allow packing of subpictures, the video components should be encoded with the following VTM parameters:

* ALF : 0
* CCALF : 0
* LMCSEnable : 0
* JointCbCr : 0
* IBC : 1
* HashME : 1
* BDPCM : 1

Moreover, it is necessary to disable the override of partition constraints as this is a header flag and should be the same for all subpictures:

* EnablePartitionConstraintsOverride : 0
* AMaxBT : 0

An example config files for VTM is provided in [config/test/muliplexer_external_packed_video](muliplexer_external_packed_video).

### Merging texture and depth bitstreams

The [VTM subpicture merger tool](https://vcgit.hhi.fraunhofer.de/jvet/VVCSoftware_VTM/-/tree/master/source/App/SubpicMergeApp) can be used to merge the subpicture bitstreams containing texture and geometry into a single bitstream.

```
$ SubpicMergeAppStatic -l settings.txt -o merged.266
```

An example of <settings.txt> would look like:

```
4096 2176 0 0 texture.266
2048 1152 4096 0 geometry.266
2048 1024 4096 1152 filler.266
```

where the first two columns indicate the dimension (width, height) of a subpicture.
The next two columns indicate the position of top-left corner (col, row) of a subpicture in a packed video frame.
The last column lists the name of a file containing the subpicture. 

### Re-writing V3C bitstream with the multiplexer

Once a packed video component is generated, the original V3C bitstream generated initially by [TMIV encoding step](http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1/-/blob/main/doc/running.md#tmiv-encoding) can be re-written and the packed video component multiplexed in V3C bitstream.  
In order for a Multiplexer to do the re-writing of VPS, a multiplexer config file must be appended with a packingInformation. An example is provided in [config/test/muliplexer_external_packed_video](muliplexer_external_packed_video).
The information about codec and information related to occupancy, geometry and attribute is copied from the original V3C bitstream as the information shall not change during the packing process.

When a config file with packingInformation is ready, multiplexer is executed with command line as described in clause ["Running the TMIV multiplexer"](http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1/-/blob/main/doc/running.md#running-the-tmiv-multiplexer).

## MIV conformance testing [ISO/IEC 23090-23]

Specifying *outputLogPath* either in a configuration file via `-c` or on the command-line using the `-p`, instructs the TMIV decoder to output a log-file for conformance testing. 

This option may be combined with other TMIV decoder actions such as rendering views. None of these actions will affect the log-file.