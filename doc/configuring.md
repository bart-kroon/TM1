# Configuring TMIV

## Basic structure of the TMIV configuration files

The TMIV configuration files use the [JSON](http://json.org) file format. The main parameters are key-value pairs of the root object, for example:

```json
{
  "intraPeriod": 32,
  "blockSizeDepthQualityDependent": [ 16, 32 ]
}
```

Selectable components follow this pattern:

```json
{
  "PrunerMethod": "HierarchicalPruner",
  "HierarchicalPruner": {
    "erode": 3,
    "etcetera": "..."
  }
}
```

In some cases there is a bypass component, e.g. `NoPruner` or `NoCuller`. Even though these have no parameters, the key still needs to be provided:

```json
{
  "PrunerMethod": "NoPruner",
  "NoPruner": { }
}
```

## Input/output parameters

TMIV constructs input and output paths using:
1. base directories `configDirectory`, `inputDirectory` and `outputDirectory`,
1. path formats with [{fmt}](https://github.com/fmtlib/fmt) syntax.

The [format syntax](https://fmt.dev/latest/syntax.html#syntax) is convenient and flexible. It is not required to use all available placeholders and with positional placeholders it is possible to use the same placeholder multiple times.

### Base directories

* **configDirectory**: the base directory for the configuration files.
* **inputDirectory**: the base directory for the input files.
* **outputDirectory**: the base directory for the output files.

An experiment may have multiple steps, e.g. Encoder, Multiplexer, Decoder, in which case the output directory of one step may correspond to the input directory of the next step.

Base directories can be overriden by using absolute paths for the path formats:
* `/my/output/dir` plus `my_filename.yuv` gives `/my/output/dir/my_filename.yuv`
* `/my/output/dir` plus `/my/path.yuv` gives `/my/path.yuv`

### Path format placeholders

Depending on the situation, the following placeholders are available to form paths:

* **Atlas ID** (e.g. 12) as read from the bitstream
* **Atlas index** in `0 .. atlas count - 1`, as determined by the test model
* **Content ID** (e.g. J for Kitchen), as specified by the `-s` command-line parameter
* **Frame height** (e.g. 1080) as determined by the test model
* **Frame index** is 0 for the Encoder and in the range `0 .. number of input frames - 1` for the Renderer
* **Frame width** (e.g. 1920) as determined by the test model
* **Number of input frames** (e.g. 97), as specified by the `-n` command-line parameter
* **Number of output frames** (e.g. 300), as specified by the `-N` command-line parameter
* **Pose trace name** (e.g. p02) as specified using the `-P` command-line parameter
* **Test ID** (e.g. R0 or QP3) as specified by the `-r` command-line parameter
* **Video format** (e.g. yuv420p10le) as determined by the test model
* **View index** in `0 .. view count - 1`, as determined by the test model
* **View name** (e.g. v11) as specified using the `-v` command-line parameter or determined by the test model

### Input path formats

Unless specified otherwise, the base directory for these path formats is `inputDirectory`.

* **inputBitstreamPathFmt**: the path format of the input bitstream, consumed by the Multiplexer and Decoder, with placeholders:
    * 0: number of input frames,
    * 1: content ID,
    * 2: test ID.
* **inputGeometryPathFmt**: the path format of the multiview uncompressed geometry (depth) data, consumed by the Encoder and Renderer, with placeholders:
    * 0: number of input frames,
    * 1: content ID,
    * 2: test ID,
    * 3: view name,
    * 4, 5, 6: frame width, frame height, and video format.
* **inputTexturePathFmt**: the path format of the multiview uncompressed texture (color) data, consumed by the Encoder and Renderer, with the same placeholders as `inputGeometryPathFormat`.
* **inputTransparencyPathFmt**: the path format of the multiview uncompressed transparency (alpha) data, with the same placeholders as `inputGeometryPathFormat`.
* **inputEntityPathFmt**: the path format of the multiview uncompressed entity maps, consumed by the Encoder, with the same placeholders as `inputGeometryPathFormat`.
* **inputGeometryVideoFramePathFmt**: the path format of the uncompresed geometry video data (GVD), consumed by the Decoder for out-of-band video decoding, e.g. for testing alternative video codecs, with placeholders:
    * 0: number of input frames,
    * 1: content ID,
    * 2: test ID,
    * 3: atlas ID,
    * 4, 5, 6: frame width, frame height and video format.
* **inputOccupancyVideoFramePathFmt**: the path format of the uncompresed occupancy video data (OVD), consumed by the Decoder for out-of-band video decoding, e.g. for testing alternative video codecs, with the same placeholders as `inputGeometryVideoFramePathFmt`.
* **inputTextureVideoFramePathFmt**: the path format of the uncompresed attribute video data (AVD) with attribute ID `ATTR_TEXTURE`, consumed by the Decoder for out-of-band video decoding, e.g. for testing alternative video codecs, with the same placeholders as `inputGeometryVideoFramePathFmt`.
* **inputPackedVideoFramePathFmt**: the path format of the uncompresed packed video data (PVD), consumed by the Decoder for out-of-band video decoding, with the same placeholders as `inputGeometryVideoFramePathFmt`.
* **inputTransparencyVideoFramePathFmt**: the path format of the uncompresed attribute video data (AVD) with attribute ID `ATTR_TRANSPARENCY`, with the same placeholders as `inputGeometryVideoFramePathFmt`.
* **inputGeometryVideoSubBitstreamPathFmt**: the path format of the geometry video sub-bitstream, consumed by the Multiplexer, with placeholders:
    * 0: number of input frames,
    * 1: content ID,
    * 2: test ID,
    * 3: atlas index.
* **inputOccupancyVideoSubBitstreamPathFmt**: the path format of the occupancy video sub-bitstream, consumed by the Multiplexer, with the same placeholders as `inputGeometryVideoSubBitstreamPathFmt`.
* **inputTextureVideoSubBitstreamPathFmt**: the path format of the attribute video sub-bitstream with attribute ID `ATTR_TEXTURE`, consumed by the Multiplexer, with placeholders:
    * 0: number of input frames,
    * 1: content ID,
    * 2: test ID,
    * 3: atlas index,
    * 4: attribute index.
* **inputPackedVideoSubBitstreamPathFmt**: the path format of the packed video sub-bitstream, consumed by the Multiplexer, with the same placeholders as `inputGeometryVideoSubBitstreamPathFmt`.
* **inputNormalVideoSubBitstreamPathFmt**: the path format of the attribute video sub-bitstream with attribute ID `ATTR_NORMAL`, consumed by the Multiplexer, with the same placeholders as `inputTextureVideoSubBitstreamPathFmt`.
* **inputTransparencyVideoSubBitstreamPathFmt**: the path format of the attribute video sub-bitstream with attribute ID `ATTR_TRANSPARENCY`, consumed by the Multiplexer, with the same placeholders as `inputTextureVideoSubBitstreamPathFmt`.
* **inputMaterialIdVideoSubBitstreamPathFmt**: the path format of the attribute video sub-bitstream with attribute ID `ATTR_MATERIAL_ID`, consumed by the Multiplexer, with the same placeholders as `inputTextureVideoSubBitstreamPathFmt`.
* **inputReflectanceVideoSubBitstreamPathFmt**: the path format of the attribute video sub-bitstream with attribute ID `ATTR_REFLECTANCE`, consumed by the Multiplexer, with the same placeholders as `inputTextureVideoSubBitstreamPathFmt`.
* **inputPoseTracePathFmt**: the path format of the pose trace CSV file, with `configDirectory` as base directory and placeholders:
    * 0: number of input frames,
    * 1: content ID,
    * 2: test ID,
    * 3: pose trace name.
* **inputSequenceConfigPathFmt**: the path format of the sequence (content) configuration file, consumed by the Encoder and Renderer to load (reconstructed) source parameters, with `inputDirectory` as primary (try first) and `configDirectory` as secondary (try next) base directory, with placeholders:
    * 0: number of input frames,
    * 1: content ID,
    * 2: test ID,
    * 3: frame index.
* **inputViewportParamsPathFmt**: the path format of the sequence (content) configuration file, to load viewport parameters, consumed by the Decoder and Renderer, with `configDirectory` as base directory and with placeholders:
    * 0: number of input frames,
    * 1: content ID,
    * 2: test ID.
* **inputMpiPcsPathFmt**: the path format of an MPI sequence in Packed Compressed Storage (PCS), i.e. texture + transparency (alpha) data, with the same placeholders as `inputGeometryPathFormat`.

### Output path formats

Unless specified otherwise, the base directory for these path formats is `outputDirectory`.

* **outputBitstreamPathFmt**: the path format of the output bitstream, produced by the Encoder and Multiplexer, with the same placeholders as `inputBitstreamPathFmt`.
* **outputBlockToPatchMapPathFmt**: the path format of the block-to-patch map output, produced by the Decoder, with placeholders:
    * 0: number of input frames,
    * 1: content ID,
    * 2: test ID,
    * 3: atlas index,
    * 4, 5, 6: frame width, frame height, and video format.
* **outputGeometryVideoDataPathFmt**: the path format of the uncompressed geometry video data (GVD), produced by the Encoder, with the same placeholders as `outputBlockToPatchMapPathFmt`.
* **outputOccupancyVideoDataPathFmt**: the path format of the uncompressed occupancy video data (OVD), produced by the Encoder, with the same placeholders as `outputBlockToPatchMapPathFmt`.
* **outputTextureVideoDataPathFmt**: the path format of the uncompressed attribute video data (AVD) with attribute ID `ATTR_TEXTURE`, produced by the Encoder, with the same placeholders as `outputBlockToPatchMapPathFmt`.
* **outputPackedVideoDataPathFmt**: the path format of the uncompressed packed video data (PVD), produced by the Encoder, with the same placeholders as `outputBlockToPatchMapPathFmt`.
* **outputTransparencyVideoDataPathFmt**: the path format of the uncompressed attribute video data (AVD) with attribute ID `ATTR_TRANSPARENCY`, with the same placeholders as `outputBlockToPatchMapPathFmt`.
* **outputMultiviewGeometryPathFmt**: the path format of the reconstructed multiview geometry (depth) data, produced by the Decoder, with placeholders:
    * 0: number of input frames,
    * 1: content ID,
    * 2: test ID,
    * 3: view index,
    * 4, 5, 6: frame width, frame height, and video format.
* **outputMultiviewOccupancyPathFmt**: the path format of the reconstructed (pruned) multiview occupancy data, produced by the Decoder, with the same placeholders as `outputMultiviewGeometryPathFmt`.
* **outputMultiviewTexturePathFmt**: the path format of the reconstructed (pruned) multiview texture (color) data, produced by the Decoder, with the same placeholders as `outputMultiviewGeometryPathFmt`.
* **outputMultiviewTransparencyPathFmt**: the path format of the reconstructed (pruned) multiview transparency (alpha) data, produced by the Decoder, with the same placeholders as `outputMultiviewGeometryPathFmt`.
* **outputSequenceConfigPathFmt**: the path format of the reconstructed sequence (content) configuration file, produced by the Decoder, written for each frame whereby parameters have changed, with the same placeholders as `inputSequenceConfigPathFmt`.
* **outputViewportGeometryPathFmt**: the path format of the geometry (depth) video data of the rendered viewport, produced by the Decoder or Renderer, with placeholders:
    * 0: number of input frames,
    * 1: content ID,
    * 2: test ID,
    * 3: number of output frames,
    * 4: view or pose trace name,
    * 5, 6, 7: frame width, frame height, and video format.
* **outputViewportTexturePathFmt**: the path format of the texture (color) video data of the rendered viewport, produced by the Decoder or Renderer, with the same placeholders as `outputViewportGeometryPathFmt`.
* **outputTexturePathFmt**: the path format of the uncompressed texture (color) video data of an MPI sequence, produced by the MPI converter, with the same placeholders as `inputGeometryPathFmt`.
* **outputTransparencyPathFmt**: the path format of the uncompressed transparency (alpha) video data of an MPI sequence, produced by the MPI converter, with the same placeholders as `inputGeometryPathFmt`.
* **outputMpiPcsPathFmt**: the path format of a compressed MPI content in PCS format, produced by the MPI converter, with the same placeholders as `inputGeometryPathFmt`.

## Profile-tier-level parameters

The Encoder accepts profile-tier-level parameters and writes these into the bitstream. The Encoder does not check these parameters.
In general the Encoder is happy to go out-of-profile to enable experiments. Similarly, the Decoder will check PTL constraints but will only warn.
The DecoderLog executable checks conformance of a bitstream, and will raise a runtime error when a bitstream is out-of-profile. 

* **toolsetIdc:** string; the `ptl_profile_toolset_idc` value, as in the specification, e.g. "MIV Main".
* **levelIdc:** string; the `ptl_level_idc` value, as a string "1.0", "1.5", etc. On the command-line, add quotes otherwise the value is interpret as a number.
* **oneV3cFrameOnly:** bool; the `ptc_one_v3c_frame_only_flag` value.
* **reconstructionIdc:** string; the `ptl_reconstruction_idc` value, as in the specification, e.g. "Rec Unconstrained".

Note that:

* The `ptc_max_decodes` value is automatically determined by the Encoder.
* The `ptc_restricted_geometry_flag` is false for the Encoder and true for the MpiEncoder.

## Algorithmic parameters

Some of the algorithmic components of the test model have parameters. This section provides a short description of these parameters in reference to the _test model_ [[4]](/README.md#references).

Some general parameters are defined in the root of the configuration file. All others are defined in the node of a component.

### General parameters

These parameters are in the root of the configuration file and may be accessed by multiple components:

* **startFrame**: int; first frame to be encoded.
  By default, the encoder will select a start frame based on the sequence configuration.
* Output video sub-bitstreams:
    * **haveOccupancyVideo:** bool; output occupancy video data (OVD) instead of  depth/occupancy coding within geometry video data (GVD). Make sure to use ExplicitOccupancy as the geometry quantizer.
    * **haveGeometryVideo:** bool; output geometry video data (GVD) to encode depth and optionally also occuapncy information. Without geometry, depth estimation is shifted from a pre-encoding to a post-decoding process.
    * **haveTextureVideo:** bool; output attribute video data (AVD) to encode the texture attribute. When false texture data is still needed as input of the test model.
    * **haveTransparencyVideo:** bool; output attribute video data (AVD) to encode the transparency attribute.
    * **bitDepthOccupancyVideo:** int; the bit depth of the occupancy video (if present).
    * **bitDepthGeometryVideo:** int; the bit depth of the geometry video (if present).
    * **bitDepthTextureVideo:** int; the bit depth of the texture video (if present).
    * **framePacking:** bool; output packed video data (PVD) to encode the packed components together in a single video frame. The bit depth is derived from the component bit depths.
    * **embeddedOccupancy:** bool; with geometry video enabled and occupancy video disabled, indicate if occupancy is encoded in geometry (true) or if occupancy is only available at patch-level (false).
    * **geometryScaleEnabledFlag:** bool; when true geometry is downscaled by a factor of two in respect to the atlas frame size. Otherwise geometry is at full resolution.
* Atlas frame size calculation and packing:
    * **intraPeriod:** int; the intra period as in video codecs, indicating the period between independent random access points (IRAP). This value dictates the intra period of the atlas bitstreams and common atlas bitstream. The video sub bitstreams need to have the same value.
    * **interPeriod:** int; the period between inter-coded common atlas frames and atlas frames, counting both IRAP and non-IRAP frames. When omitted, the inter period will equal the intra period, such that all frames are IRAP frames.
    * **maxAtlases:** int; the maximum number of transmitted atlases in total.
    * **maxEntityId:** int; the maximum ID of entities, whereby the default "0" disables entity-based coding.
    * **maxLumaSamplerate:** float; the maximum number of luma samples per second counted over all atlases, groups and components. This parameter communicates the equally-named CTC constraint.
    * **maxLumaPictureSize:** int; the maximum number of samples per atlas frame, which corresponds to the maximum number of samples per texture attribute video frame. This parameter communicates the equally-named CTC constraint.
    * **numGroups:** int; the number of groups of atlases:
        * 0: No grouping, each atlas is independently useful for rendering (as with the MIV view anchor)
        * 1: Single group, all atlases are needed for rendering (as with the MIV anchor)
        * 2: _N_ groups, with the group-based encoder, partitions views to independently encode multiple groups of atlases.
* Metadata:
    * **dqParamsPresentFlag:** bool; optional parameter, when false the depth quantization parameters are not written to the bitstream.
    * **randomAccess:** bool; when true complete MIV metadata (inclding VPS, CAD, AD V3C units) are written per IRAP to the bitstream such that there is no dependency when parsing metadata between different IRAPs.

### Depth quality assessor

* **blendingFactor:** float; for every reprojected pixel it is checked if
  reprojected geometry value is higher than 1.0 - _blendingFactor_ of geometry
  value of collocated pixel or any of its neighbors in the target view.
* **maxOutlierRatio:** float; pixel outlier threshold above which the geometry
  quality is judged to be low.

### Encoder

Most of the parameters are defined in the root. The exception is:

* **dilate:** int; number of dilation steps on the aggregated pruning mask.
  This parameter is only in effect for low depth quality.

### Geometry quantizer

* **depthOccThresholdIfSet:** float; the value of the depth-occupancy map
  threshold times 2<sup>-geoBitDepth</sup>, in case occupancy information is encoded in the geometry video data of
  a specific view.

### Explicit occupancy

* **occupancyScale:** int[2]; the downscale factor of occupancy maps applied
  across all atlases when occupancy is signaled explicitly.

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

* **minPatchSize:** int; is the number of pixels of the smallest border of the
  patch, below which the patch is discarded. Default value is 8.
* **overlap:** int; is the number of pixels which will be added to a frontier
  of a newly split patch; it prevents seam artefacts. Default value is 1.
* **enablePatchInPatch:** bool; enable the patch-in-patch extension on the MaxRect algorithm.
  It allows the insertion of patches into other patches. Default value is 1.
* **enableMerging:** bool; enable the patch merging step.
* **sortingMethod:** int; code for the sorting method of clusters during packing
  step in [0 (by descending area), 1 (by increasing view index)].
* **enableRecursiveSplit:** bool; enable the recursive split of clusters.

### Basic view allocator

* **maxBasicViewFraction:** float; the maximum number of available luma samples
  that is used for coding views completely.
* **outputAdditionalViews:** bool; when true output basic views and additional
  views. when false output only basic views.
* **minNonCodedViews:** int; the minimum number of source views that will not
  be coded as basic view.

### Server-side Inpainter

* **SubMethod:** string; underlying ViewOptimizerMethod.
* **resolution:** int[2]; resolution [hor,vert] of the inpainted view.
* **SynthesizerMethod:** string; the sythesizer used for creating an inpainted background view.
* **InpainterMethod:** string; the actual inpainting method used to inpaint the hidden regions.
* **blurRadius:** int; radius of the blur kernel used to locate regions of missing data.
* **inpaintThreshold:** int; a comparison threshold used to locate regions of missing data.
* **fieldOfViewMargin:** float; the field-of-view of the inpainted view is the accumulative field-of-view of the source-views but expanded with this factor.

### Decoder

* **geometryEdgeMagnitudeTh:** int; parameter of the geometry upscaling, in
  line with the hypothetical reference renderer.
* **maxCurvature:** int;  parameter of the geometry upscaling, in line with the
  hypothetical reference renderer.
* **minForegroundConfidence:** float; parameter of the geometry upscaling, in
  line with the hypothetical reference renderer.

### AdditiveSynthesizer

* **maxStretching:** float; used to discard too large triangles that yield large gaps in the target view.
* **depthParameter:** float; blending parameter that prioritizes foreground over background. A negative value blends background over foreground.
* **rayAngleParameter:** float; blending parameter that prioritizes on ray-angle.
* **stretchingParameter:** float; blending parameter that prioritizes on triangle-stretching (de-occlusion).

### ViewWeightingSynthesizer

* **angularScaling:** float; Drives the splat size at the warping stage.
* **minimalWeight:** float; Allows for splat degeneracy test at the warping stage.
* **stretchFactor:** float; Limits the splat max size at the warping stage.
* **overloadFactor:** float; Geometry selection parameter at the selection stage.
* **filteringPass:** int; Number of median filtering pass to apply to the visibility map.
* **blendingFactor:** float; Used to control the blending at the shading stage.

### MPI encoder

* **textureDilation:** int; Number of dilations steps for the the texture atlas.
* **transparencyDynamic:** int; The number of transparency levels in the transparency atlas is given by pow(2, transparencyDynamic).

### MPI synthesizer

* **minAlpha:** float; Drives the blending process at shading stage.


### Multi-tile

* **numberPartitionCols:** int; number of partition rows in a atlas, and it should be at least 1.
* **numberPartitionRows:** int; number of partition columns in a atlas, and it should be at least 1.
* **singlePartitionPerTileFlag:** bool; a tile includes multiple partitions. When true, tiles have only one partition. When false, a tile may include more than one partition. It shall be true in this project.
* **partitionWidth:** int[numberPartitionCols]; the width of the partition in the ith row, i=0,1,..numberPartitionCols-1. And the sum of partitionWidth shall be equal to the width of atlas. The values of partitionWidth shall be a multiple of 64, except for the last element. For example, if numberPartitionCols=1 and the atlas width=1920, then partitionWidth=[1920].
* **partitionHeight:** int[numberPartitionRows]; the height of the partition in the jth column (j=0,1,...numberPartitionRows-1). The sum height of partitionHeight shall be equal to the height of atlas. The values of partitionHeight shall a multiple of 64, except for the last element. For example, if numberPartitionRows=5 and the atlas height=4640, then partitionHeight=[1088,1088,1088,1088,288].

* **toolsetIdc:** shall be "MIV Extended" if using multi-tile.


