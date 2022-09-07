#ifndef TMIV_MIVBITSTREAM_TILE_H
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto TilePartition::partitionPosX() const noexcept { return m_partitionPosX; }

constexpr auto TilePartition::partitionPosY() const noexcept { return m_partitionPosY; }

constexpr auto TilePartition::partitionWidth() const noexcept { return m_partitionWidth; }

constexpr auto TilePartition::partitionHeight() const noexcept { return m_partitionHeight; }

inline auto TilePartition::partitionPatchList() const -> const MivBitstream::PatchParamsList & {
  return m_patchList;
}

constexpr auto TilePartition::partitionPosX(int32_t value) noexcept -> TilePartition & {
  m_partitionPosX = value;
  return *this;
}

constexpr auto TilePartition::partitionPosY(int32_t value) noexcept -> TilePartition & {
  m_partitionPosY = value;
  return *this;
}

constexpr auto TilePartition::partitionWidth(int32_t value) noexcept -> TilePartition & {
  m_partitionWidth = value;
  return *this;
}

constexpr auto TilePartition::partitionHeight(int32_t value) noexcept -> TilePartition & {
  m_partitionHeight = value;
  return *this;
}

inline auto TilePartition::partitionPatchList(MivBitstream::PatchParamsList values) noexcept
    -> TilePartition & {
  m_patchList = std::move(values);
  return *this;
}
inline auto TilePartition::addPatchToTile(MivBitstream::PatchParams value) -> TilePartition & {
  m_patchList.emplace_back(value);
  return *this;
}

} // namespace TMIV::MivBitstream
