/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

static constexpr auto srcJson0 = R"({"gas_qs":1,"gas_log2_bw_minus2":8
,"gas_num_views_minus1":0
,"view_idx_0":{"gas_projection_plane_height_minus1":1079
,"gas_projection_plane_width_minus1":1919
,"blocks":[
[
 {"gas_split_flag":1, "gas_quad_split_flag":0, "gas_split_orientation_flag":1, "gas_split_symmetry_flag":0, "gas_split_first_block_bigger":0 ,"subblks": [{ "gas_skip_flag": 0 ,"gas_zmin_delta":265 ,"gas_zmax_delta":289},{ "gas_skip_flag": 0 ,"gas_zmin_delta":-76 ,"gas_zmax_delta":-24}]}
, {"gas_split_flag":1, "gas_quad_split_flag":0, "gas_split_orientation_flag":1, "gas_split_symmetry_flag":1 ,"subblks": [{ "gas_skip_flag": 0 ,"gas_zmin_delta":-53 ,"gas_zmax_delta":-76},{ "gas_skip_flag": 0 ,"gas_zmin_delta":-28 ,"gas_zmax_delta":-53}]}
]
,[
 {"gas_split_flag":0 ,"subblks": [{ "gas_skip_flag": 0 ,"gas_zmin_delta":-81 ,"gas_zmax_delta":-153}]}
, {"gas_split_flag":0 ,"subblks": [{ "gas_skip_flag": 0 ,"gas_ltmin_flag":1 ,"gas_zmin_delta":0 ,"gas_ltmax_flag":0 ,"gas_zmax_delta":0}]}
]
]}
})";
static constexpr auto srcJson1 = R"({"gas_qs":1,"gas_log2_bw_minus2":8
,"gas_num_views_minus1":0
,"view_idx_0":{"gas_projection_plane_height_minus1":1079
,"gas_projection_plane_width_minus1":1919
,"blocks":[
[
 {"gas_split_flag":0 ,"subblks": [{ "gas_skip_flag": 1}]}, {"gas_split_flag":0 ,"subblks": [{ "gas_skip_flag": 1}]}]
,[
 {"gas_split_flag":0 ,"subblks": [{ "gas_skip_flag": 1}]}, {"gas_split_flag":0 ,"subblks": [{ "gas_skip_flag": 1}]}]
]}
})";
static constexpr auto insertedLog = R"(ssvh_unit_size_precision_bytes_minus1=2

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
v3c_unit(64)
vuh_unit_type=V3C_CAD
vuh_v3c_parameter_set_id=0
ssnh_unit_size_precision_bytes_minus1=0

----------------------------------------------------------------------------------------------------
nal_unit_type=NAL_CASPS
nal_layer_id=0
nal_temporal_id_plus1=1
NumBytesInRbsp=3
casps_common_atlas_sequence_parameter_set_id=0
casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4=0
casps_extension_present_flag=true
casps_miv_extension_present_flag=true
casps_extension_7bits=0
casme_depth_low_quality_flag=false
casme_depth_quantization_params_present_flag=false
casme_vui_params_present_flag=false

----------------------------------------------------------------------------------------------------
nal_unit_type=NAL_CAF_IDR
nal_layer_id=0
nal_temporal_id_plus1=1
NumBytesInRbsp=50
caf_common_atlas_sequence_parameter_set_id=0
caf_common_atlas_frm_order_cnt_lsb=0
caf_extension_present_flag=true
caf_miv_extension_present_flag=true
caf_extension_7bits=0
miv_view_params_list=mvp_num_views_minus1=0
mvp_explicit_view_id_flag=false
ce_view_pos_x[ 0 ]=0.0
ce_view_pos_y[ 0 ]=0.0
ce_view_pos_z[ 0 ]=0.0
ce_view_quat_x[ 0 ]=0
ce_view_quat_y[ 0 ]=0
ce_view_quat_z[ 0 ]=0
mvp_inpaint_flag[ 0 ]=false
mvp_intrinsic_params_equal_flag=true
ci_cam_type[ 0 ]=equirectangular
ci_projection_plane_width_minus1[ 0 ]=1919
ci_projection_plane_height_minus1[ 0 ]=1079
ci_erp_phi_min[ 0 ]=0
ci_erp_phi_max[ 0 ]=0
ci_erp_theta_min[ 0 ]=0
ci_erp_theta_max[ 0 ]=0
mvp_pruning_graph_params_present_flag=true
pp_is_root_flag[ 0 ]=true

====================================================================================================
v3c_unit(51)
vuh_unit_type=V3C_AD
vuh_v3c_parameter_set_id=0
vuh_atlas_id=0
ssnh_unit_size_precision_bytes_minus1=2

----------------------------------------------------------------------------------------------------
nal_unit_type=NAL_SUFFIX_NSEI
nal_layer_id=0
nal_temporal_id_plus1=1
NumBytesInRbsp=41
payloadType=geometry_assistance
payloadSize=28
gas_qs=1
gas_num_views_minus1=0
gas_log2_bw_minus2=8
# VIEWIDX 0
gas_projection_plane_height_minus1[0]=1079
gas_projection_plane_width_minus1[0]=1919
block y=0 x=0 gas_split_flag=true gas_quad_split_flag=false gas_split_orientation_flag=true gas_split_symmetry_flag=false gas_split_first_block_bigger=false [gas_skip_flag=false,gas_zmin_delta=265,gas_zmax_delta=289] [gas_skip_flag=false,gas_zmin_delta=-76,gas_zmax_delta=-24]
block y=0 x=1 gas_split_flag=true gas_quad_split_flag=false gas_split_orientation_flag=true gas_split_symmetry_flag=true [gas_skip_flag=false,gas_zmin_delta=-53,gas_zmax_delta=-76] [gas_skip_flag=false,gas_zmin_delta=-28,gas_zmax_delta=-53]
block y=1 x=0 gas_split_flag=false [gas_skip_flag=false,gas_zmin_delta=-81,gas_zmax_delta=-153]
block y=1 x=1 gas_split_flag=false [gas_skip_flag=false,gas_ltmin_flag=true,gas_ltmax_flag=false,gas_zmin_delta=0,gas_zmax_delta=0]
payloadType=geometry_assistance
payloadSize=8
gas_qs=1
gas_num_views_minus1=0
gas_log2_bw_minus2=8
# VIEWIDX 0
gas_projection_plane_height_minus1[0]=1079
gas_projection_plane_width_minus1[0]=1919
block y=0 x=0 gas_split_flag=false [gas_skip_flag=true]
block y=0 x=1 gas_split_flag=false [gas_skip_flag=true]
block y=1 x=0 gas_split_flag=false [gas_skip_flag=true]
block y=1 x=1 gas_split_flag=false [gas_skip_flag=true]

====================================================================================================
v3c_unit(5)
vuh_unit_type=V3C_AD
vuh_v3c_parameter_set_id=0
vuh_atlas_id=0
ssnh_unit_size_precision_bytes_minus1=2

====================================================================================================
)";
