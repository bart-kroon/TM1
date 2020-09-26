/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
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

#include "test.h"

#include <TMIV/MivBitstream/SceneObjectInformation.h>

#include <utility>

namespace TMIV::MivBitstream {
namespace {
auto makeSoiVisibilityCones(std::uint16_t value) -> SoiVisibilityCones {
  SoiVisibilityCones result{};
  result.soi_direction_x = value++;
  result.soi_direction_y = value++;
  result.soi_direction_z = value++;
  result.soi_angle = value;
  return result;
}

auto make3dBoundingBox(std::uint16_t value) -> BoundingBox3D {
  BoundingBox3D result{};
  result.soi_3d_bounding_box_x = value++;
  result.soi_3d_bounding_box_y = value++;
  result.soi_3d_bounding_box_z = value++;
  result.soi_3d_bounding_box_size_x = value++;
  result.soi_3d_bounding_box_size_y = value++;
  result.soi_3d_bounding_box_size_z = value;
  return result;
}

std::vector<SceneObjectUpdate> makeUpdates(std::size_t soi_num_object_updates,
                                           bool soi_simple_objects_flag) {
  auto updates{std::vector<SceneObjectUpdate>(soi_num_object_updates)};
  std::generate(updates.begin(), updates.end(),
                [soi_object_idx = 0, soi_simple_objects_flag]() mutable {
                  SceneObjectUpdate update{};
                  update.soi_object_idx = soi_object_idx;
                  update.soi_object_cancel_flag = false;
                  if (!soi_simple_objects_flag) {
                    update.soi_object_label_update_flag = true;
                    update.soi_object_label_idx = soi_object_idx;
                    update.soi_priority_update_flag = true;
                    update.soi_priority_value = soi_object_idx / 2;
                    update.soi_object_hidden_flag = true;
                    update.soi_object_dependency_update_flag = true;
                    update.soi_object_dependency_idx = std::vector<std::size_t>(2);
                    update.soi_visibility_cones_update_flag = true;
                    update.m_soi_visibility_cones = makeSoiVisibilityCones(soi_object_idx);
                    update.soi_3d_bounding_box_update_flag = true;
                    update.soi_3d_bounding_box = make3dBoundingBox(soi_object_idx);
                    update.soi_collision_shape_update_flag = true;
                    update.soi_collision_shape_id = 2 * soi_object_idx;
                    update.soi_point_style_update_flag = true;
                    update.soi_point_shape_id = 4 * soi_object_idx;
                    update.soi_point_size = 8 * soi_object_idx;
                  }
                  ++soi_object_idx;
                  return update;
                });
  return updates;
}

auto makeSceneObjectInformation(bool soi_persistence_flag, bool soi_reset_flag,
                                bool soi_simple_objects_flag, std::size_t soi_num_object_updates,
                                std::uint8_t soi_log2_max_object_idx_updated_minus1,
                                std::uint8_t soi_3d_bounding_box_scale_log2 = 0,
                                std::uint8_t soi_log2_max_object_dependency_idx = 0)
    -> SceneObjectInformation {
  SceneObjectInformation soi{};
  soi.soi_persistence_flag(soi_persistence_flag);
  soi.soi_reset_flag(soi_reset_flag);
  soi.soi_simple_objects_flag(soi_simple_objects_flag);
  soi.soi_log2_max_object_idx_updated_minus1(soi_log2_max_object_idx_updated_minus1);
  if (!soi_simple_objects_flag) {
    soi.soi_3d_bounding_box_scale_log2(soi_3d_bounding_box_scale_log2);
    soi.soi_log2_max_object_dependency_idx(soi_log2_max_object_dependency_idx);
  }
  soi.setSceneObjectUpdates(makeUpdates(soi_num_object_updates, soi_simple_objects_flag));
  return soi;
}
} // namespace

TEST_CASE("scene_object_information", "[Scene Object Information SEI payload syntax]") {
  SECTION("Default constructor") {
    const SceneObjectInformation unit{};
    REQUIRE(toString(unit) == R"(soi_persistence_flag=false
soi_reset_flag=false
soi_num_object_updates=0
)");
    const std::size_t expected_number_of_bits = 1    // soi_persistence_flag
                                                + 1  // soi_reset_flag
                                                + 1; // soi_num_object_updates
    REQUIRE(bitCodingTest(unit, expected_number_of_bits));
  }

  std::size_t expected_number_of_bits =
      1    // soi_persistence_flag
      + 1  // soi_reset_flag
      + 1  // soi_simple_objects_flag
      + 9  // soi_object_label_present_flag ... soi_extension_present_flag
      + 5; // soi_log2_max_object_idx_updated_minus1

  SECTION("Custom fields, simple objects") {
    const auto unit{makeSceneObjectInformation(false, true, true, 4, 2)};
    REQUIRE(toString(unit) == R"(soi_persistence_flag=false
soi_reset_flag=true
soi_num_object_updates=4
soi_simple_objects_flag=true
soi_object_label_present_flag=false
soi_priority_present_flag=false
soi_object_hidden_present_flag=false
soi_object_dependency_present_flag=false
soi_visibility_cones_present_flag=false
soi_3d_bounding_box_present_flag=false
soi_collision_shape_present_flag=false
soi_point_style_present_flag=false
soi_material_id_present_flag=false
soi_extension_present_flag=false
soi_log2_max_object_idx_updated_minus1=2
soi_object_idx=0
soi_object_cancel_flag(0)=false
soi_object_idx=1
soi_object_cancel_flag(1)=false
soi_object_idx=2
soi_object_cancel_flag(2)=false
soi_object_idx=3
soi_object_cancel_flag(3)=false
)");
    expected_number_of_bits += 5          // soi_num_object_updates
                               + (4 *     // soi_num_object_updates
                                  (3      // soi_object_idx
                                   + 1)); // soi_object_cancel_flag

    REQUIRE(bitCodingTest(unit, expected_number_of_bits));
  }

  SECTION("Custom fields, complex objects") {
    const auto unit{makeSceneObjectInformation(true, false, false, 2, 1, 1, 2)};
    REQUIRE(toString(unit) == R"(soi_persistence_flag=true
soi_reset_flag=false
soi_num_object_updates=2
soi_simple_objects_flag=false
soi_object_label_present_flag=true
soi_priority_present_flag=true
soi_object_hidden_present_flag=true
soi_object_dependency_present_flag=true
soi_visibility_cones_present_flag=true
soi_3d_bounding_box_present_flag=true
soi_collision_shape_present_flag=true
soi_point_style_present_flag=true
soi_material_id_present_flag=true
soi_extension_present_flag=true
soi_3d_bounding_box_scale_log2=1
soi_log2_max_object_idx_updated_minus1=1
soi_log2_max_object_dependency_idx=2
soi_object_idx=0
soi_object_cancel_flag(0)=false
soi_object_label_update_flag(0)=true
soi_object_label_idx(0)=0
soi_priority_update_flag(0)=true
soi_priority_value(0)=0
soi_object_hidden_flag(0)=true
soi_object_dependency_update_flag(0)=true
soi_object_num_dependencies(0)=2
soi_object_dependency_idx(0)=0
soi_object_dependency_idx(0)=0
soi_visibility_cones_update_flag(0)=true
soi_direction_x(0)=0
soi_direction_y(0)=1
soi_direction_z(0)=2
soi_angle(0)=3
soi_3d_bounding_box_update_flag(0)=true
soi_3d_bounding_box_x(0)=0
soi_3d_bounding_box_y(0)=1
soi_3d_bounding_box_z(0)=2
soi_3d_bounding_box_size_x(0)=3
soi_3d_bounding_box_size_y(0)=4
soi_3d_bounding_box_size_z(0)=5
soi_collision_shape_update_flag(0)=true
soi_collision_shape_id(0)=0
soi_point_style_update_flag(0)=true
soi_point_shape_id(0)=0
soi_point_size(0)=0
soi_object_idx=1
soi_object_cancel_flag(1)=false
soi_object_label_update_flag(1)=true
soi_object_label_idx(1)=1
soi_priority_update_flag(1)=true
soi_priority_value(1)=0
soi_object_hidden_flag(1)=true
soi_object_dependency_update_flag(1)=true
soi_object_num_dependencies(1)=2
soi_object_dependency_idx(1)=0
soi_object_dependency_idx(1)=0
soi_visibility_cones_update_flag(1)=true
soi_direction_x(1)=1
soi_direction_y(1)=2
soi_direction_z(1)=3
soi_angle(1)=4
soi_3d_bounding_box_update_flag(1)=true
soi_3d_bounding_box_x(1)=1
soi_3d_bounding_box_y(1)=2
soi_3d_bounding_box_z(1)=3
soi_3d_bounding_box_size_x(1)=4
soi_3d_bounding_box_size_y(1)=5
soi_3d_bounding_box_size_z(1)=6
soi_collision_shape_update_flag(1)=true
soi_collision_shape_id(1)=2
soi_point_style_update_flag(1)=true
soi_point_shape_id(1)=4
soi_point_size(1)=8
)");
    expected_number_of_bits += 3           // soi_num_object_updates
                               + 5         // soi_3d_bounding_box_scale_log2
                               + 5         // soi_log2_max_object_dependency_idx
                               + (2 *      // soi_num_object_updates
                                  (2       // soi_object_idx
                                   + 1     // soi_object_cancel_flag
                                   + 1     // soi_object_update_label_flag
                                   + 1     // soi_object_label_idx
                                   + 1     // soi_priority_update_flag
                                   + 1     // soi_priority_update_flag
                                   + 4     // soi_priority_value
                                   + 1     // soi_object_hidden_flag
                                   + 1     // soi_object_dependency_update_flag
                                   + 4     // soi_object_num_dependencies
                                   + 2 * 2 // soi_object_dependency_idx
                                   + 1     // soi_visibility_cones_update_flag
                                   + 16    // soi_direction_x
                                   + 16    // soi_direction_y
                                   + 16    // soi_direction_z
                                   + 16    // soi_angle
                                   + 1     // soi_3d_bounding_box_update_flag
                                   + 6 * 4 // soi_3d_bounding_box position and size fields
                                   + 1     // soi_collision_shape_update_flag
                                   + 16    // soi_collision_shape_id
                                   + 1     // soi_point_style_update_flag
                                   + 8     // soi_point_shape_id
                                   + 16    // soi_point_size
                                   ));
    REQUIRE(bitCodingTest(unit, expected_number_of_bits));
  }
  // TODO add test with some fields set false. This is required for such a comple structure
}
} // namespace TMIV::MivBitstream
