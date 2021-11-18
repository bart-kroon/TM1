/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

static constexpr auto referenceHlsLog = R"(ssvh_unit_size_precision_bytes_minus1=2

====================================================================================================
v3c_unit(18)
vuh_unit_type=V3C_VPS
ptl_tier_flag=false
ptl_profile_codec_group_idc=AVC Progressive High
ptl_profile_toolset_idc=V-PCC Basic
ptl_profile_reconstruction_idc=Rec0
ptl_max_decodes_idc=unconstrained
ptl_level_idc=[unknown:0]
ptl_num_sub_profiles=0
ptl_extended_sub_profile_flag=false
ptl_toolset_constraints_present_flag=false
vps_v3c_parameter_set_id=0
vps_atlas_count_minus1=0
vps_atlas_id[ 0 ]=0
vps_frame_width[ 0 ]=0
vps_frame_height[ 0 ]=0
vps_map_count_minus1[ 0 ]=0
vps_auxiliary_video_present_flag[ 0 ]=false
vps_occupancy_video_present_flag[ 0 ]=false
vps_geometry_video_present_flag[ 0 ]=false
vps_attribute_video_present_flag[ 0 ]=false
vps_extension_present_flag=false

====================================================================================================
v3c_unit(53)
vuh_unit_type=V3C_AD
vuh_v3c_parameter_set_id=0
vuh_atlas_id=0
ssnh_unit_size_precision_bytes_minus1=0

----------------------------------------------------------------------------------------------------
nal_unit_type=NAL_ASPS
nal_layer_id=0
nal_temporal_id_plus1=1
NumBytesInRbsp=5
asps_atlas_sequence_parameter_set_id=0
asps_frame_width=0
asps_frame_height=0
asps_geometry_3d_bit_depth_minus1=0
asps_geometry_2d_bit_depth_minus1=0
asps_log2_max_atlas_frame_order_cnt_lsb_minus4=0
asps_max_dec_atlas_frame_buffering_minus1=0
asps_long_term_ref_atlas_frames_flag=false
asps_num_ref_atlas_frame_lists_in_asps=0
asps_use_eight_orientations_flag=false
asps_extended_projection_enabled_flag=false
asps_normal_axis_limits_quantization_enabled_flag=false
asps_normal_axis_max_delta_value_enabled_flag=false
asps_patch_precedence_order_flag=false
asps_log2_patch_packing_block_size=0
asps_patch_size_quantizer_present_flag=false
asps_map_count_minus1=0
asps_pixel_deinterleaving_enabled_flag=false
asps_raw_patch_enabled_flag=false
asps_eom_patch_enabled_flag=false
asps_plr_enabled_flag=false
asps_vui_parameters_present_flag=false
asps_extension_present_flag=false

----------------------------------------------------------------------------------------------------
nal_unit_type=NAL_AFPS
nal_layer_id=0
nal_temporal_id_plus1=1
NumBytesInRbsp=2
afps_atlas_frame_parameter_set_id=0
afps_atlas_sequence_parameter_set_id=0
afti_single_tile_in_atlas_frame_flag=true
afti_signalled_tile_id_flag=false
afps_output_flag_present_flag=false
afps_num_ref_idx_default_active_minus1=0
afps_additional_lt_afoc_lsb_len=0
afps_lod_mode_enabled_flag=false
afps_raw_3d_offset_bit_count_explicit_mode_flag=false
afps_extension_present_flag=false

----------------------------------------------------------------------------------------------------
nal_unit_type=NAL_AUD
nal_layer_id=0
nal_temporal_id_plus1=1
NumBytesInRbsp=1
aframe_type=I_TILE

----------------------------------------------------------------------------------------------------
nal_unit_type=NAL_V3C_AUD
nal_layer_id=0
nal_temporal_id_plus1=1
NumBytesInRbsp=1
aframe_type=I_TILE

----------------------------------------------------------------------------------------------------
nal_unit_type=NAL_EOS
nal_layer_id=0
nal_temporal_id_plus1=1
NumBytesInRbsp=0

----------------------------------------------------------------------------------------------------
nal_unit_type=NAL_EOB
nal_layer_id=0
nal_temporal_id_plus1=1
NumBytesInRbsp=0

----------------------------------------------------------------------------------------------------
nal_unit_type=NAL_FD
nal_layer_id=0
nal_temporal_id_plus1=1
NumBytesInRbsp=0

----------------------------------------------------------------------------------------------------
nal_unit_type=NAL_AAPS
nal_layer_id=0
nal_temporal_id_plus1=1
NumBytesInRbsp=1
aaps_atlas_adaptation_parameter_set_id=0
aaps_log2_max_afoc_present_flag=false
aaps_extension_present_flag=false

----------------------------------------------------------------------------------------------------
nal_unit_type=NAL_CASPS
nal_layer_id=0
nal_temporal_id_plus1=1
NumBytesInRbsp=1
casps_common_atlas_sequence_parameter_set_id=0
casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4=0
casps_extension_present_flag=false

----------------------------------------------------------------------------------------------------
nal_unit_type=NAL_CAF_TRIAL
nal_layer_id=0
nal_temporal_id_plus1=1
NumBytesInRbsp=2
caf_common_atlas_sequence_parameter_set_id=0
caf_common_atlas_frm_order_cnt_lsb=0
caf_extension_present_flag=false

----------------------------------------------------------------------------------------------------
nal_unit_type=NAL_CAF_IDR
nal_layer_id=0
nal_temporal_id_plus1=1
NumBytesInRbsp=2
caf_common_atlas_sequence_parameter_set_id=0
caf_common_atlas_frm_order_cnt_lsb=0
caf_extension_present_flag=false

====================================================================================================
)";
