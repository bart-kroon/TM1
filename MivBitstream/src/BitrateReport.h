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

#ifndef _TMIV_MIVBITSTREAM_BITRATEREPORT_H_
#define _TMIV_MIVBITSTREAM_BITRATEREPORT_H_

#include <TMIV/MivBitstream/MivDecoder.h>

#include <map>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
class StatisticalVariable {
public:
  auto &operator<<(size_t value) {
    ++m_count;
    m_sum += value;
    return *this;
  }

  friend auto operator<<(ostream &stream, const StatisticalVariable &x) -> ostream & {
    auto average =
        x.m_count > 0 ? double(x.m_sum) / x.m_count : numeric_limits<double>::quiet_NaN();
    return stream << x.m_count << ',' << x.m_sum << ',' << average;
  }

private:
  size_t m_count{};
  size_t m_sum{};
};

struct CompareVuh {
  auto operator()(const VpccUnitHeader &vuh1, const VpccUnitHeader &vuh2) const -> bool {
    if (vuh1.vuh_unit_type() != vuh2.vuh_unit_type()) {
      return vuh1.vuh_unit_type() < vuh2.vuh_unit_type();
    }
    if (vuh1.vuh_unit_type() == VuhUnitType::VPCC_VPS) {
      return false;
    }
    if (vuh1.vuh_atlas_id() != vuh2.vuh_atlas_id()) {
      return vuh1.vuh_atlas_id() < vuh2.vuh_atlas_id();
    }
    if (vuh1.vuh_unit_type() != VuhUnitType::VPCC_GVD &&
        vuh1.vuh_unit_type() != VuhUnitType::VPCC_AVD) {
      return false;
    }
    if (vuh1.vuh_map_index() != vuh2.vuh_map_index()) {
      return vuh1.vuh_map_index() < vuh2.vuh_map_index();
    }
    if (vuh1.vuh_unit_type() != VuhUnitType::VPCC_AVD) {
      return false;
    }
    return vuh1.vuh_attribute_index() < vuh2.vuh_attribute_index();
  }
};

struct CompareNuh {
  auto operator()(const NalUnitHeader &nuh1, const NalUnitHeader &nuh2) const -> bool {
    if (nuh1.nal_unit_type() != nuh2.nal_unit_type()) {
      return nuh1.nal_unit_type() < nuh2.nal_unit_type();
    }
    if (nuh1.nal_layer_id() != nuh2.nal_layer_id()) {
      return nuh1.nal_layer_id() < nuh2.nal_layer_id();
    }
    return nuh1.nal_temporal_id_plus1() < nuh2.nal_temporal_id_plus1();
  }
};

class BitrateReport {
public:
  void printTo(ostream &stream) {
    stream << "vuh_unit_type,vuh_atlas_id,vuh_map_index,vuh_attribute_index,count,sum,average\n";
    for (const auto &[vuh, stats] : m_vuhStats) {
      stream << vuh.vuh_unit_type() << ',';
      switch (vuh.vuh_unit_type()) {
      case VuhUnitType::VPCC_VPS:
        stream << ",,";
        break;
      case VuhUnitType::VPCC_AD:
      case VuhUnitType::VPCC_OVD:
        stream << int(vuh.vuh_atlas_id()) << ",,";
        break;
      case VuhUnitType::VPCC_GVD:
        stream << int(vuh.vuh_atlas_id()) << ',' << int(vuh.vuh_map_index()) << ',';
        break;
      case VuhUnitType::VPCC_AVD:
        stream << int(vuh.vuh_atlas_id()) << ',' << int(vuh.vuh_map_index()) << ','
               << int(vuh.vuh_attribute_index());
        break;
      default:
        abort();
      }
      stream << ',' << stats << '\n';
    }

    stream << ",,,,,,\n";
    stream << "nal_unit_type,nal_layer_id,nal_temporal_id,,count,sum,average\n";

    for (const auto &[nuh, stats] : m_nuhStats) {
      stream << nuh.nal_unit_type() << ',' << int(nuh.nal_layer_id()) << ','
             << (nuh.nal_temporal_id_plus1() - 1) << ",," << stats << '\n';
    }
  }

  void add(const VpccUnitHeader &vuh, size_t size) { m_vuhStats[vuh] << size; }
  auto add(const NalUnitHeader &nuh, size_t size) { m_nuhStats[nuh] << size; }

private:
  map<VpccUnitHeader, StatisticalVariable, CompareVuh> m_vuhStats;
  map<NalUnitHeader, StatisticalVariable, CompareNuh> m_nuhStats;
};
} // namespace TMIV::MivBitstream

#endif