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

#include <TMIV/IO/IvMetadataWriter.h>

#include <TMIV/IO/IO.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::IO {
IvMetadataWriter::IvMetadataWriter(const Json &config, const string &baseDirectoryField,
                                   const string &fileNameField) {
  m_path = getFullPath(config, baseDirectoryField, fileNameField);
  m_stream.open(m_path, ios::binary);
  if (!m_stream.good()) {
    ostringstream what;
    what << "Failed to open metadata file " << m_path;
    throw runtime_error(what.str());
  }
}

void IvMetadataWriter::writeIvSequenceParams(IvSequenceParams ivSequenceParams) {
  m_ivSequenceParams = move(ivSequenceParams);
  m_ivSequenceParams.encodeTo(m_bitstream);
}

void IvMetadataWriter::writeIvAccessUnitParams(IvAccessUnitParams ivAccessUnitParams) {
  fprintf(stdout, "1\n");
  const bool skipAtlasParamsList =
      m_ivAccessUnitParams.atlasParamsList == ivAccessUnitParams.atlasParamsList;
  fprintf(stdout, "2\n");
  m_ivAccessUnitParams = ivAccessUnitParams;
  fprintf(stdout, "3: przed if skipAtlasParamsList\n");
  if (skipAtlasParamsList) {
    fprintf(stdout, "\t3.1 w if\n");
    ivAccessUnitParams.atlasParamsList.reset();
    fprintf(stdout, "\t3.2 w if\n");
  }
  fprintf(stdout, "4: po if skipAtlasParamsList\n");
  ivAccessUnitParams.encodeTo(m_bitstream, m_ivSequenceParams);
  fprintf(stdout, "5: po encodeTo\n");
}
} // namespace TMIV::IO
