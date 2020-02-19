/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

#include <TMIV/IO/IvMetadataReader.h>

#include <TMIV/IO/IO.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::IO {
IvMetadataReader::IvMetadataReader(const Json &config, const string &baseDirectoryField,
                                   const string &fileNameField) {
  m_path = getFullPath(config, baseDirectoryField, fileNameField);
  m_stream.open(m_path, ios::binary);
  if (!m_stream.good()) {
    ostringstream what;
    what << "Failed to open metadata file " << m_path;
    throw runtime_error(what.str());
  }
}

void IvMetadataReader::readIvSequenceParams() {
  // TODO(BK): Implement
}

void IvMetadataReader::readIvAccessUnitParams() {
  const auto currentAtlasParamsList = m_ivAccessUnitParams.atlasParamsList;
  m_ivAccessUnitParams = IvAccessUnitParams::decodeFrom(m_bitstream, m_ivSequenceParams);
  if (!m_ivAccessUnitParams.atlasParamsList) {
    m_ivAccessUnitParams.atlasParamsList = currentAtlasParamsList.value();
  }
  // TODO(BK): Partial atlas information? (With only some atlas_id's present.)
}

auto IvMetadataReader::readAccessUnit(int accessUnit) -> bool {
  if (m_accessUnit == accessUnit) {
    return false;
  }
  m_accessUnit = accessUnit;
  m_bitstream.reset();
  m_stream.seekg(0);
  readIvSequenceParams();
  for (int i = 0; i <= accessUnit; ++i) {
    readIvAccessUnitParams();
  }
  return true;
}
} // namespace TMIV::IO
