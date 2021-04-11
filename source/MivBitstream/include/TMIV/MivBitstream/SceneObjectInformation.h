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

#ifndef TMIV_MIVBITSTREAM_SCENEOBJECTINFORMATION_H
#define TMIV_MIVBITSTREAM_SCENEOBJECTINFORMATION_H

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
  uint16_t soi_angle{};
};

struct Soi3dBoundingBox {
  [[nodiscard]] constexpr auto operator==(const Soi3dBoundingBox &other) const noexcept -> bool {
    return (soi_3d_bounding_box_x == other.soi_3d_bounding_box_x) &&
           (soi_3d_bounding_box_y == other.soi_3d_bounding_box_y) &&
           (soi_3d_bounding_box_z == other.soi_3d_bounding_box_z) &&
           (soi_3d_bounding_box_size_x == other.soi_3d_bounding_box_size_x) &&
           (soi_3d_bounding_box_size_y == other.soi_3d_bounding_box_size_y) &&
           (soi_3d_bounding_box_size_z == other.soi_3d_bounding_box_size_z);
  }

  size_t soi_3d_bounding_box_x{};
  size_t soi_3d_bounding_box_y{};
  size_t soi_3d_bounding_box_z{};
  size_t soi_3d_bounding_box_size_x{};
  size_t soi_3d_bounding_box_size_y{};
  size_t soi_3d_bounding_box_size_z{};
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
           (soi_visibility_cones == other.soi_visibility_cones) &&
           (soi_3d_bounding_box_update_flag == other.soi_3d_bounding_box_update_flag) &&
           (soi_3d_bounding_box == other.soi_3d_bounding_box) &&
           (soi_collision_shape_update_flag == other.soi_collision_shape_update_flag) &&
           (soi_collision_shape_id == other.soi_collision_shape_id) &&
           (soi_point_style_update_flag == other.soi_point_style_update_flag) &&
           (soi_point_shape_id == other.soi_point_shape_id) &&
           (soi_point_size == other.soi_point_size) &&
           (soi_material_id_update_flag == other.soi_material_id_update_flag) &&
           (soi_material_id == other.soi_material_id);
  }
  size_t soi_object_idx{};
  bool soi_object_cancel_flag{};
  std::optional<bool> soi_object_label_update_flag{};
  std::optional<size_t> soi_object_label_idx{};
  std::optional<bool> soi_priority_update_flag{};
  std::optional<uint8_t> soi_priority_value{};
  std::optional<bool> soi_object_hidden_flag{};
  std::optional<bool> soi_object_dependency_update_flag{};
  std::vector<size_t> soi_object_dependency_idx{};
  std::optional<bool> soi_visibility_cones_update_flag{};
  std::optional<SoiVisibilityCones> soi_visibility_cones{};
  std::optional<bool> soi_3d_bounding_box_update_flag{};
  std::optional<Soi3dBoundingBox> soi_3d_bounding_box{};
  std::optional<bool> soi_collision_shape_update_flag{};
  std::optional<uint16_t> soi_collision_shape_id{};
  std::optional<bool> soi_point_style_update_flag{};
  std::optional<uint8_t> soi_point_shape_id{};
  std::optional<uint16_t> soi_point_size{};
  std::optional<bool> soi_material_id_update_flag{};
  std::optional<uint16_t> soi_material_id{};
};

// 23090-12: scene_object_information ( payloadSize )
class SceneObjectInformation {
public:
  [[nodiscard]] auto soi_persistence_flag() const noexcept -> bool;
  [[nodiscard]] auto soi_reset_flag() const noexcept -> bool;
  [[nodiscard]] auto soi_num_object_updates() const noexcept -> size_t;
  [[nodiscard]] auto soi_simple_objects_flag() const -> bool;
  [[nodiscard]] auto soi_object_label_present_flag() const -> bool;
  [[nodiscard]] auto soi_priority_present_flag() const -> bool;
  [[nodiscard]] auto soi_object_hidden_present_flag() const -> bool;
  [[nodiscard]] auto soi_object_dependency_present_flag() const -> bool;
  [[nodiscard]] auto soi_visibility_cones_present_flag() const -> bool;
  [[nodiscard]] auto soi_3d_bounding_box_present_flag() const -> bool;
  [[nodiscard]] auto soi_collision_shape_present_flag() const -> bool;
  [[nodiscard]] auto soi_point_style_present_flag() const -> bool;
  [[nodiscard]] auto soi_material_id_present_flag() const -> bool;
  [[nodiscard]] auto soi_extension_present_flag() const -> bool;
  [[nodiscard]] auto soi_3d_bounding_box_scale_log2() const -> uint8_t;
  [[nodiscard]] auto soi_log2_max_object_idx_updated_minus1() const -> uint8_t;
  [[nodiscard]] auto soi_log2_max_object_dependency_idx() const -> uint8_t;
  [[nodiscard]] auto soi_object_idx(size_t i) const -> size_t;
  [[nodiscard]] auto soi_object_cancel_flag(size_t k) const -> bool;
  [[nodiscard]] auto soi_object_label_update_flag(size_t k) const -> bool;
  [[nodiscard]] auto soi_object_label_idx(size_t k) const -> size_t;
  [[nodiscard]] auto soi_priority_update_flag(size_t k) const -> bool;
  [[nodiscard]] auto soi_priority_value(size_t k) const -> uint8_t;
  [[nodiscard]] auto soi_object_hidden_flag(size_t k) const -> bool;
  [[nodiscard]] auto soi_object_dependency_update_flag(size_t k) const -> bool;
  [[nodiscard]] auto soi_object_num_dependencies(size_t k) const -> uint8_t;
  [[nodiscard]] auto soi_object_dependency_idx(size_t k, size_t j) const -> size_t;
  [[nodiscard]] auto soi_visibility_cones_update_flag(size_t k) const -> bool;
  [[nodiscard]] auto soi_direction_x(size_t k) const -> std::int16_t;
  [[nodiscard]] auto soi_direction_y(size_t k) const -> std::int16_t;
  [[nodiscard]] auto soi_direction_z(size_t k) const -> std::int16_t;
  [[nodiscard]] auto soi_angle(size_t k) const -> uint16_t;
  [[nodiscard]] auto soi_3d_bounding_box_update_flag(size_t k) const -> bool;
  [[nodiscard]] auto soi_3d_bounding_box_x(size_t k) const -> size_t;
  [[nodiscard]] auto soi_3d_bounding_box_y(size_t k) const -> size_t;
  [[nodiscard]] auto soi_3d_bounding_box_z(size_t k) const -> size_t;
  [[nodiscard]] auto soi_3d_bounding_box_size_x(size_t k) const -> size_t;
  [[nodiscard]] auto soi_3d_bounding_box_size_y(size_t k) const -> size_t;
  [[nodiscard]] auto soi_3d_bounding_box_size_z(size_t k) const -> size_t;
  [[nodiscard]] auto soi_collision_shape_update_flag(size_t k) const -> bool;
  [[nodiscard]] auto soi_collision_shape_id(size_t k) const -> uint16_t;
  [[nodiscard]] auto soi_point_style_update_flag(size_t k) const -> bool;
  [[nodiscard]] auto soi_point_shape_id(size_t k) const -> uint8_t;
  [[nodiscard]] auto soi_point_size(size_t k) const -> uint16_t;
  [[nodiscard]] auto soi_material_id_update_flag(size_t k) const -> bool;
  [[nodiscard]] auto soi_material_id(size_t k) const -> uint16_t;

  constexpr auto soi_persistence_flag(bool value) noexcept -> auto &;
  constexpr auto soi_reset_flag(bool value) noexcept -> auto &;
  constexpr auto soi_num_object_updates(size_t value) noexcept -> auto &;
  constexpr auto soi_simple_objects_flag(bool value) noexcept -> auto &;
  constexpr auto soi_object_label_present_flag(bool value) noexcept -> auto &;
  constexpr auto soi_priority_present_flag(bool value) noexcept -> auto &;
  constexpr auto soi_object_hidden_present_flag(bool value) noexcept -> auto &;
  constexpr auto soi_object_dependency_present_flag(bool value) noexcept -> auto &;
  constexpr auto soi_visibility_cones_present_flag(bool value) noexcept -> auto &;
  constexpr auto soi_3d_bounding_box_present_flag(bool value) noexcept -> auto &;
  constexpr auto soi_collision_shape_present_flag(bool value) noexcept -> auto &;
  constexpr auto soi_point_style_present_flag(bool value) noexcept -> auto &;
  constexpr auto soi_material_id_present_flag(bool value) noexcept -> auto &;
  constexpr auto soi_extension_present_flag(bool value) noexcept -> auto &;
  constexpr auto soi_3d_bounding_box_scale_log2(uint8_t value) noexcept -> auto &;
  constexpr auto soi_log2_max_object_idx_updated_minus1(uint8_t value) noexcept -> auto &;
  constexpr auto soi_log2_max_object_dependency_idx(uint8_t value) noexcept -> auto &;
  auto setSceneObjectUpdates(std::vector<SceneObjectUpdate> &&updates) noexcept -> void;

  friend auto operator<<(std::ostream &stream, const SceneObjectInformation &x) -> std::ostream &;

  auto operator==(const SceneObjectInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> SceneObjectInformation;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  [[nodiscard]] auto isUpdateValid(size_t k) const -> bool;
  [[nodiscard]] auto isBoundingBoxValid(size_t k) const -> bool;

  bool m_soi_persistence_flag{};
  bool m_soi_reset_flag{};
  std::optional<size_t> m_temporary_soi_num_object_updates{};
  std::optional<bool> m_soi_simple_objects_flag{};
  std::optional<bool> m_soi_object_label_present_flag{};
  std::optional<bool> m_soi_priority_present_flag{};
  std::optional<bool> m_soi_object_hidden_present_flag{};
  std::optional<bool> m_soi_object_dependency_present_flag{};
  std::optional<bool> m_soi_visibility_cones_present_flag{};
  std::optional<bool> m_soi_3d_bounding_box_present_flag{};
  std::optional<bool> m_soi_collision_shape_present_flag{};
  std::optional<bool> m_soi_point_style_present_flag{};
  std::optional<bool> m_soi_material_id_present_flag{};
  std::optional<bool> m_soi_extension_present_flag{};
  std::optional<uint8_t> m_soi_3d_bounding_box_scale_log2{};
  uint8_t m_soi_log2_max_object_idx_updated_minus1{};
  std::optional<uint8_t> m_soi_log2_max_object_dependency_idx{};
  std::vector<SceneObjectUpdate> m_object_updates{};
};
} // namespace TMIV::MivBitstream

#include <TMIV/MivBitstream/SceneObjectInformation.hpp>

#endif
