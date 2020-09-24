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
#include <ostream>
#include <vector>

namespace TMIV::MivBitstream {

struct SceneObjectUpdate {
  auto operator==(const SceneObjectUpdate &other) const noexcept -> bool {
    return m_soi_object_idx == other.m_soi_object_idx; // TODO complete
  }
  std::uint8_t m_soi_object_idx{};
};

class SceneObjectUpdates {
public:
  //   operator bool?
  [[nodiscard]] auto has_value() const noexcept -> bool { return !m_object_updates.empty(); }

  [[nodiscard]] auto soi_num_object_updates() const noexcept -> std::size_t {
    return m_object_updates.size();
  }

  [[nodiscard]] auto soi_simple_objects_flag() const noexcept -> bool {
    return m_soi_simple_objects_flag;
  }

  auto soi_num_object_updates(const std::size_t value) noexcept -> void {
    m_object_updates = std::vector<SceneObjectUpdate>(value);
  }

  auto operator==(const SceneObjectUpdates &other) const noexcept -> bool {
    return (m_soi_simple_objects_flag == other.m_soi_simple_objects_flag) &&
           (m_object_updates == other.m_object_updates);
  }

  constexpr auto soi_simple_objects_flag(const bool value) noexcept -> void {
    m_soi_simple_objects_flag = value;
  }

private:
  bool m_soi_simple_objects_flag{};
  std::vector<SceneObjectUpdate> m_object_updates{};
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
  auto soi_num_object_updates(const std::size_t value) noexcept -> auto & {
    m_object_updates.soi_num_object_updates(value);
    return *this;
  }
  constexpr auto soi_simple_objects_flag(const bool value) noexcept -> auto & {
    m_object_updates.soi_simple_objects_flag(value);
    return *this;
  }

  friend auto operator<<(std::ostream &stream, const SceneObjectInformation &x) -> std::ostream &;

  auto operator==(const SceneObjectInformation &other) const noexcept -> bool;
  auto operator!=(const SceneObjectInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> SceneObjectInformation;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  bool m_soi_persistence_flag{};
  bool m_soi_reset_flag{};
  SceneObjectUpdates m_object_updates{};
};
} // namespace TMIV::MivBitstream

#endif
