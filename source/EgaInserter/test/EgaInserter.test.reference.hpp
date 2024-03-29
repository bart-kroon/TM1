/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

static constexpr auto srcJson0 = R"(
{
   "ega_num_views_minus1":0,
   "ega_num_available_assistance_types_minus1":0,
   "view_idx_0":{
      "ega_assistance_present_flag":1,
      "ega_type_idx_0":{
         "ega_assistance_type_present_flag":1,
         "bbgf_qs":1,
         "bbgf_log2_bw_minus2":8,
         "bbgf_max_number_of_splits":1,
         "bbgf_projection_plane_height_minus1":1079,
         "bbgf_projection_plane_width_minus1":1919,
         "blocks":[
            [
               {
                  "bbgf_split_flag":1,
                  "bbgf_quad_split_flag":0,
                  "bbgf_split_orientation_flag":1,
                  "bbgf_split_symmetry_flag":0,
                  "bbgf_split_first_block_bigger":0,
                  "subblks":[
                     {
                        "bbgf_skip_flag":0,
                        "bbgf_zmin_delta":265,
                        "bbgf_zmax_delta":289
                     },
                     {
                        "bbgf_skip_flag":0,
                        "bbgf_zmin_delta":-76,
                        "bbgf_zmax_delta":-24
                     }
                  ]
               },
               {
                  "bbgf_split_flag":1,
                  "bbgf_quad_split_flag":0,
                  "bbgf_split_orientation_flag":1,
                  "bbgf_split_symmetry_flag":1,
                  "subblks":[
                     {
                        "bbgf_skip_flag":0,
                        "bbgf_zmin_delta":-53,
                        "bbgf_zmax_delta":-76
                     },
                     {
                        "bbgf_skip_flag":0,
                        "bbgf_zmin_delta":-28,
                        "bbgf_zmax_delta":-53
                     }
                  ]
               }
            ],
            [
               {
                  "bbgf_split_flag":0,
                  "bbgf_skip_flag":0,
                  "bbgf_zmin_delta":-81,
                  "bbgf_zmax_delta":-153
               },
               {
                  "bbgf_split_flag":0,
                  "bbgf_skip_flag":0,
                  "bbgf_ltmin_flag":1,
                  "bbgf_zmin_delta":0,
                  "bbgf_ltmax_flag":0,
                  "bbgf_zmax_delta":0
               }
            ]
         ]
      }
   }
}
)";
static constexpr auto srcJson1 = R"(
{
   "ega_num_views_minus1":0,
   "ega_num_available_assistance_types_minus1":0,
   "view_idx_0":{
      "ega_assistance_present_flag":1,
      "ega_type_idx_0":{
         "ega_assistance_type_present_flag":1,
         "bbgf_qs":1,
         "bbgf_log2_bw_minus2":8,
         "bbgf_max_number_of_splits":1,
         "bbgf_projection_plane_height_minus1":1079,
         "bbgf_projection_plane_width_minus1":1919,
         "blocks":[
            [
               {
                  "bbgf_split_flag":0,
                  "bbgf_skip_flag":1
               },
               {
                  "bbgf_split_flag":0,
                  "bbgf_skip_flag":1
               }
            ],
            [
               {
                  "bbgf_split_flag":0,
                  "bbgf_skip_flag":1
               },
               {
                  "bbgf_split_flag":0,
                  "bbgf_skip_flag":1
               }
            ]
         ]
      }
   }
}
)";
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
vps_map_absolute_coding_enabled_flag[ 0 ][ 0 ]=true
vps_map_predictor_index_diff[ 0 ][ 0 ]=0
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
casps_miv_2_extension_present_flag=false
casps_extension_6bits=0
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
ce_view_pos_x[ 0 ]=0
ce_view_pos_y[ 0 ]=0
ce_view_pos_z[ 0 ]=0
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
cs_u_min[ 0 ]=0
cs_u_max[ 0 ]=0
cs_v_min[ 0 ]=0
cs_v_max[ 0 ]=0
mvp_view_background_flag[ 0 ]=false

====================================================================================================
v3c_unit(53)
vuh_unit_type=V3C_AD
vuh_v3c_parameter_set_id=0
vuh_atlas_id=0
ssnh_unit_size_precision_bytes_minus1=2

----------------------------------------------------------------------------------------------------
nal_unit_type=NAL_PREFIX_NSEI
nal_layer_id=0
nal_temporal_id_plus1=1
NumBytesInRbsp=43
payloadType=extended_geometry_assistance
ega_num_views_minus1=0
ega_num_available_assistance_types_minus1=0
# VIEWIDX 0
ega_assistance_present_flag[0]=true
# EGATYPEIDX 0
ega_assistance_type_present_flag[0][0]=true
bbgf_qs[0]=1
bbgf_log2_bw_minus2[0]=8
bbgf_max_number_of_splits[0]=1
bbgf_projection_plane_height_minus1[0]=1079
bbgf_projection_plane_width_minus1[0]=1919
block y=0 x=0 bbgf_split_flag=true bbgf_quad_split_flag=false bbgf_split_orientation_flag=true bbgf_split_symmetry_flag=false bbgf_split_first_block_bigger=false [ bbgf_skip_flag=false bbgf_zmin_delta=265 bbgf_zmax_delta=289 ] [ bbgf_skip_flag=false bbgf_zmin_delta=-76 bbgf_zmax_delta=-24 ]
block y=0 x=1 bbgf_split_flag=true bbgf_quad_split_flag=false bbgf_split_orientation_flag=true bbgf_split_symmetry_flag=true [ bbgf_skip_flag=false bbgf_zmin_delta=-53 bbgf_zmax_delta=-76 ] [ bbgf_skip_flag=false bbgf_zmin_delta=-28 bbgf_zmax_delta=-53 ]
block y=1 x=0 bbgf_split_flag=false bbgf_skip_flag=false bbgf_zmin_delta=-81 bbgf_zmax_delta=-153
block y=1 x=1 bbgf_split_flag=false bbgf_skip_flag=false bbgf_ltmin_flag=true bbgf_ltmax_flag=false bbgf_zmin_delta=0 bbgf_zmax_delta=0
payloadType=extended_geometry_assistance
ega_num_views_minus1=0
ega_num_available_assistance_types_minus1=0
# VIEWIDX 0
ega_assistance_present_flag[0]=true
# EGATYPEIDX 0
ega_assistance_type_present_flag[0][0]=true
bbgf_qs[0]=1
bbgf_log2_bw_minus2[0]=8
bbgf_max_number_of_splits[0]=1
bbgf_projection_plane_height_minus1[0]=1079
bbgf_projection_plane_width_minus1[0]=1919
block y=0 x=0 bbgf_split_flag=false bbgf_skip_flag=true
block y=0 x=1 bbgf_split_flag=false bbgf_skip_flag=true
block y=1 x=0 bbgf_split_flag=false bbgf_skip_flag=true
block y=1 x=1 bbgf_split_flag=false bbgf_skip_flag=true

====================================================================================================
v3c_unit(5)
vuh_unit_type=V3C_AD
vuh_v3c_parameter_set_id=0
vuh_atlas_id=0
ssnh_unit_size_precision_bytes_minus1=2

====================================================================================================
)";
