#ifndef TMIV_MIVBITSTREAM_TILE_H
#define TMIV_MIVBITSTREAM_TILE_H

#include "TMIV/MivBitstream/PatchParamsList.h"

namespace TMIV::MivBitstream {
class TilePartition {
public:
  [[nodiscard]] constexpr auto partitionPosX() const noexcept;
  [[nodiscard]] constexpr auto partitionPosY() const noexcept;
  [[nodiscard]] constexpr auto partitionWidth() const noexcept;
  [[nodiscard]] constexpr auto partitionHeight() const noexcept;
  [[nodiscard]] inline auto partitionPatchList() const -> const MivBitstream::PatchParamsList &;

  constexpr auto partitionPosX(int32_t value) noexcept -> TilePartition &;
  constexpr auto partitionPosY(int32_t value) noexcept -> TilePartition &;
  constexpr auto partitionWidth(int32_t value) noexcept -> TilePartition &;
  constexpr auto partitionHeight(int32_t value) noexcept -> TilePartition &;
  inline auto partitionPatchList(MivBitstream::PatchParamsList values) noexcept -> TilePartition &;
  inline auto addPatchToTile(MivBitstream::PatchParams value) -> TilePartition &;

private:
  int32_t m_partitionPosX{};
  int32_t m_partitionWidth{};
  int32_t m_partitionPosY{};
  int32_t m_partitionHeight{};
  MivBitstream::PatchParamsList m_patchList;
};
using TileParamsList = std::vector<std::vector<TilePartition>>;
} // namespace TMIV::MivBitstream

#include "Tile.hpp"

#endif
