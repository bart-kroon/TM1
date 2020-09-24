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
auto makeSceneObjectUpdates(bool soi_simple_objects_flag, std::size_t soi_num_object_updates,
                            std::uint8_t soi_log2_max_object_idx_updated_minus1,
                            std::uint8_t soi_3d_bounding_box_scale_log2 = 0,
                            std::uint8_t soi_log2_max_object_dependency_idx = 0)
    -> SceneObjectUpdates {
  SceneObjectUpdates updates{};
  updates.soi_simple_objects_flag = soi_simple_objects_flag;
  updates.soi_num_object_updates(soi_num_object_updates);
  updates.soi_3d_bounding_box_scale_log2 = soi_3d_bounding_box_scale_log2;
  updates.soi_log2_max_object_idx_updated_minus1 = soi_log2_max_object_idx_updated_minus1;
  updates.soi_log2_max_object_dependency_idx = soi_log2_max_object_dependency_idx;
  std::generate(updates.m_object_updates.begin(), updates.m_object_updates.end(),
                [soi_object_idx = 0, soi_object_cancel_flag = false]() mutable {
                  SceneObjectUpdate update{};
                  update.soi_object_idx = soi_object_idx++;
                  update.soi_object_cancel_flag = soi_object_cancel_flag;
                  soi_object_cancel_flag = !soi_object_cancel_flag;
                  return update;
                });
  return updates;
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

  SceneObjectInformation unit{};
  std::size_t expected_number_of_bits =
      1    // soi_persistence_flag
      + 1  // soi_reset_flag
      + 1  // soi_simple_objects_flag
      + 9  // soi_object_label_present_flag ... soi_extension_present_flag
      + 5; // soi_log2_max_object_idx_updated_minus1

  SECTION("Custom fields, simple objects") {
    unit.soi_persistence_flag(false).soi_reset_flag(true);
    unit.setSceneObjectUpdates(makeSceneObjectUpdates(true, 4, 2));
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
soi_object_cancel_flag=false
soi_object_idx=1
soi_object_cancel_flag=true
soi_object_idx=2
soi_object_cancel_flag=false
soi_object_idx=3
soi_object_cancel_flag=true
)");
    expected_number_of_bits += 5       // soi_num_object_updates
                               + (4 *  // soi_num_object_updates
                                  (3   // soi_object_idx
                                   + 1 // soi_object_cancel_flag
                                   ));
    REQUIRE(bitCodingTest(unit, expected_number_of_bits));
  }

  SECTION("Custom fields, complex objects") {
    unit.soi_persistence_flag(true).soi_reset_flag(false);
    unit.setSceneObjectUpdates(makeSceneObjectUpdates(false, 2, 1, 1, 2));
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
soi_object_cancel_flag=false
soi_object_idx=1
soi_object_cancel_flag=true
)");
    expected_number_of_bits += 3       // soi_num_object_updates
                               + 5     // soi_3d_bounding_box_scale_log2
                               + 5     // soi_log2_max_object_dependency_idx
                               + (2 *  // soi_num_object_updates
                                  (2   // soi_object_idx
                                   + 1 // soi_object_cancel_flag
                                   ));
    REQUIRE(bitCodingTest(unit, expected_number_of_bits));
  }
}
} // namespace TMIV::MivBitstream
