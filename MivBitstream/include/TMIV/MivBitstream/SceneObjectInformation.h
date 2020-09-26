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

#ifndef _TMIV_MIVBITSTREAM_SCENEOBJECTINFORMATION_H_
#define _TMIV_MIVBITSTREAM_SCENEOBJECTINFORMATION_H_

#include <TMIV/Common/Bitstream.h>

#include <cstdint>
#include <optional>
#include <ostream>
#include <utility>
#include <vector>

namespace TMIV::MivBitstream {

struct SoiVisibilityCones {
  [[nodiscard]] constexpr auto operator==(const SoiVisibilityCones &other) const noexcept -> bool {
    return (soi_direction_x == other.soi_direction_x) &&
           (soi_direction_y == other.soi_direction_y) &&
           (soi_direction_z == other.soi_direction_z) && (soi_angle == other.soi_angle);
  }

  std::int16_t soi_direction_x{};
  std::int16_t soi_direction_y{};
  std::int16_t soi_direction_z{};
  std::uint16_t soi_angle{};
};

struct BoundingBox3D {
  [[nodiscard]] constexpr auto operator==(const BoundingBox3D &other) const noexcept -> bool {
    return (soi_3d_bounding_box_x == other.soi_3d_bounding_box_x) &&
           (soi_3d_bounding_box_y == other.soi_3d_bounding_box_y) &&
           (soi_3d_bounding_box_z == other.soi_3d_bounding_box_z) &&
           (soi_3d_bounding_box_size_x == other.soi_3d_bounding_box_size_x) &&
           (soi_3d_bounding_box_size_y == other.soi_3d_bounding_box_size_y) &&
           (soi_3d_bounding_box_size_z == other.soi_3d_bounding_box_size_z);
  }

  std::size_t soi_3d_bounding_box_x{};
  std::size_t soi_3d_bounding_box_y{};
  std::size_t soi_3d_bounding_box_z{};
  std::size_t soi_3d_bounding_box_size_x{};
  std::size_t soi_3d_bounding_box_size_y{};
  std::size_t soi_3d_bounding_box_size_z{};
};

struct SceneObjectUpdate {
  auto operator==(const SceneObjectUpdate &other) const noexcept -> bool {
    return (soi_object_idx == other.soi_object_idx) &&
           (soi_object_cancel_flag == other.soi_object_cancel_flag) &&
           (soi_object_label_update_flag == other.soi_object_label_update_flag) &&
           (soi_object_label_idx == other.soi_object_label_idx) &&
           (soi_priority_update_flag == other.soi_priority_update_flag) &&
           (soi_priority_value == other.soi_priority_value) &&
           (soi_object_hidden_flag == other.soi_object_hidden_flag) &&
           (soi_object_dependency_update_flag == other.soi_object_dependency_update_flag) &&
           (soi_object_dependency_idx == other.soi_object_dependency_idx) &&
           (soi_visibility_cones_update_flag == other.soi_visibility_cones_update_flag) &&
           (soi_3d_bounding_box_update_flag == other.soi_3d_bounding_box_update_flag) &&
           (soi_3d_bounding_box == other.soi_3d_bounding_box) &&
           (soi_collision_shape_update_flag == other.soi_collision_shape_update_flag) &&
           (soi_collision_shape_id == other.soi_collision_shape_id) &&
           (soi_point_style_update_flag == other.soi_point_style_update_flag) &&
           (soi_point_shape_id == other.soi_point_shape_id) &&
           (soi_point_size == other.soi_point_size); // TODO check if complete
  }
  std::size_t soi_object_idx{};
  bool soi_object_cancel_flag{};
  std::optional<bool> soi_object_label_update_flag{};
  std::optional<std::size_t> soi_object_label_idx{};
  std::optional<bool> soi_priority_update_flag{};
  std::optional<std::uint8_t> soi_priority_value{};
  std::optional<bool> soi_object_hidden_flag{};
  std::optional<bool> soi_object_dependency_update_flag{};
  std::vector<std::size_t> soi_object_dependency_idx{};
  std::optional<bool> soi_visibility_cones_update_flag{};
  std::optional<SoiVisibilityCones> m_soi_visibility_cones{};
  std::optional<bool> soi_3d_bounding_box_update_flag{};
  std::optional<BoundingBox3D> soi_3d_bounding_box{};
  std::optional<bool> soi_collision_shape_update_flag{};
  std::optional<std::uint16_t> soi_collision_shape_id{};
  std::optional<bool> soi_point_style_update_flag{};
  std::optional<std::uint8_t> soi_point_shape_id{};
  std::optional<std::uint16_t> soi_point_size{};
};

// 23090-12: scene_object_information ( payloadSize )
class SceneObjectInformation {
public:
  [[nodiscard]] auto soi_persistence_flag() const noexcept -> bool;
  [[nodiscard]] auto soi_reset_flag() const noexcept -> bool;
  [[nodiscard]] auto soi_num_object_updates() const noexcept -> std::size_t;
  [[nodiscard]] auto soi_simple_objects_flag() const noexcept -> bool;
  [[nodiscard]] auto soi_object_label_present_flag() const noexcept -> bool;
  [[nodiscard]] auto soi_priority_present_flag() const noexcept -> bool;
  [[nodiscard]] auto soi_object_hidden_present_flag() const noexcept -> bool;
  [[nodiscard]] auto soi_object_dependency_present_flag() const noexcept -> bool;
  [[nodiscard]] auto soi_visibility_cones_present_flag() const noexcept -> bool;
  [[nodiscard]] auto soi_3d_bounding_box_present_flag() const noexcept -> bool;
  [[nodiscard]] auto soi_collision_shape_present_flag() const noexcept -> bool;
  [[nodiscard]] auto soi_point_style_present_flag() const noexcept -> bool;
  [[nodiscard]] auto soi_material_id_present_flag() const noexcept -> bool;
  [[nodiscard]] auto soi_extension_present_flag() const noexcept -> bool;
  [[nodiscard]] auto soi_3d_bounding_box_scale_log2() const noexcept -> std::uint8_t;
  [[nodiscard]] auto soi_log2_max_object_idx_updated_minus1() const noexcept -> std::uint8_t;
  [[nodiscard]] auto soi_log2_max_object_dependency_idx() const noexcept -> std::uint8_t;
  [[nodiscard]] auto soi_object_idx(std::size_t i) const noexcept -> std::uint8_t;
  [[nodiscard]] auto soi_object_cancel_flag(std::size_t k) const noexcept -> bool;
  [[nodiscard]] auto soi_object_label_update_flag(std::size_t k) const noexcept -> bool;
  [[nodiscard]] auto soi_object_label_idx(std::size_t k) const noexcept -> std::size_t;
  [[nodiscard]] auto soi_priority_update_flag(std::size_t k) const noexcept -> bool;
  [[nodiscard]] auto soi_priority_value(std::size_t k) const noexcept -> std::uint8_t;
  [[nodiscard]] auto soi_object_hidden_flag(std::size_t k) const noexcept -> bool;
  [[nodiscard]] auto soi_object_dependency_update_flag(std::size_t k) const noexcept -> bool;
  [[nodiscard]] auto soi_object_num_dependencies(std::size_t k) const noexcept -> std::uint8_t;
  [[nodiscard]] auto soi_object_dependency_idx(std::size_t k, std::size_t j) const noexcept
      -> std::uint8_t;
  [[nodiscard]] auto soi_visibility_cones_update_flag(std::size_t k) const noexcept -> bool;
  [[nodiscard]] auto soi_direction_x(std::size_t k) const noexcept -> std::int16_t;
  [[nodiscard]] auto soi_direction_y(std::size_t k) const noexcept -> std::int16_t;
  [[nodiscard]] auto soi_direction_z(std::size_t k) const noexcept -> std::int16_t;
  [[nodiscard]] auto soi_angle(std::size_t k) const noexcept -> std::uint16_t;
  [[nodiscard]] auto soi_3d_bounding_box_update_flag(std::size_t k) const noexcept -> bool;
  [[nodiscard]] auto soi_3d_bounding_box_x(std::size_t k) const noexcept -> std::size_t;
  [[nodiscard]] auto soi_3d_bounding_box_y(std::size_t k) const noexcept -> std::size_t;
  [[nodiscard]] auto soi_3d_bounding_box_z(std::size_t k) const noexcept -> std::size_t;
  [[nodiscard]] auto soi_3d_bounding_box_size_x(std::size_t k) const noexcept -> std::size_t;
  [[nodiscard]] auto soi_3d_bounding_box_size_y(std::size_t k) const noexcept -> std::size_t;
  [[nodiscard]] auto soi_3d_bounding_box_size_z(std::size_t k) const noexcept -> std::size_t;
  [[nodiscard]] auto soi_collision_shape_update_flag(std::size_t k) const noexcept -> bool;
  [[nodiscard]] auto soi_collision_shape_id(std::size_t k) const noexcept -> std::uint16_t;
  [[nodiscard]] auto soi_point_style_update_flag(std::size_t k) const noexcept -> bool;
  [[nodiscard]] auto soi_point_shape_id(std::size_t k) const noexcept -> std::uint8_t;
  [[nodiscard]] auto soi_point_size(std::size_t k) const noexcept -> std::uint16_t;
  [[nodiscard]] auto soi_material_id_update_flag(std::size_t k) const noexcept -> bool;
  [[nodiscard]] auto soi_material_id(std::size_t k) const noexcept -> std::uint16_t;

  constexpr auto soi_persistence_flag(const bool value) noexcept -> auto & {
    m_soi_persistence_flag = value;
    return *this;
  }
  constexpr auto soi_reset_flag(const bool value) noexcept -> auto & {
    m_soi_reset_flag = value;
    return *this;
  }
  auto soi_num_object_updates(const std::uint8_t value) noexcept -> auto & {
    m_temporary_soi_num_object_updates = value;
    return *this;
  }
  constexpr auto soi_simple_objects_flag(const bool value) noexcept -> auto & {
    m_soi_simple_objects_flag = value;
    return *this;
  }
  constexpr auto soi_3d_bounding_box_scale_log2(const std::uint8_t value) noexcept -> auto & {
    m_soi_3d_bounding_box_scale_log2 = value;
    return *this;
  }
  constexpr auto soi_log2_max_object_idx_updated_minus1(const std::uint8_t value) noexcept
      -> auto & {
    m_soi_log2_max_object_idx_updated_minus1 = value;
    return *this;
  }
  constexpr auto soi_log2_max_object_dependency_idx(const std::uint8_t value) noexcept -> auto & {
    m_soi_log2_max_object_dependency_idx = value;
    return *this;
  }
  auto setSceneObjectUpdates(std::vector<SceneObjectUpdate> &&updates) noexcept -> void {
    m_object_updates = std::move(updates);
    m_temporary_soi_num_object_updates.reset();
  }

  [[nodiscard]] auto isUpdateValid(std::size_t k) const noexcept -> bool {
    return soi_num_object_updates() > 0 && k < soi_num_object_updates() &&
           !soi_object_cancel_flag(k);
  }
  [[nodiscard]] auto isBoundingBoxValid(std::size_t k) const noexcept -> bool {
    return isUpdateValid(k) && soi_3d_bounding_box_present_flag() &&
           soi_3d_bounding_box_update_flag(k) && m_object_updates[k].soi_3d_bounding_box;
  }

  friend auto operator<<(std::ostream &stream, const SceneObjectInformation &x) -> std::ostream &;

  auto operator==(const SceneObjectInformation &other) const noexcept -> bool;
  auto operator!=(const SceneObjectInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> SceneObjectInformation;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  bool m_soi_persistence_flag{};
  bool m_soi_reset_flag{};
  std::optional<std::size_t> m_temporary_soi_num_object_updates{};
  std::optional<bool> m_soi_simple_objects_flag{};
  std::optional<std::uint8_t> m_soi_3d_bounding_box_scale_log2{};
  std::uint8_t m_soi_log2_max_object_idx_updated_minus1{};
  std::optional<std::uint8_t> m_soi_log2_max_object_dependency_idx{};
  std::vector<SceneObjectUpdate> m_object_updates{};
};
} // namespace TMIV::MivBitstream

#endif
