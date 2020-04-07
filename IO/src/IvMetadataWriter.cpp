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

#include <TMIV/IO/IvMetadataWriter.h>

#include <TMIV/IO/IO.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::IO {
auto bitstreamPath(const Json &config) -> string {
  return getFullPath(config, "OutputDirectory", "AtlasMetadataPath");
}

IvMetadataWriter::IvMetadataWriter(const Json &config)
    : m_stream{bitstreamPath(config), ios::binary} {
  if (!m_stream.good()) {
    ostringstream what;
    what << "Failed to open \"" << bitstreamPath(config) << "\" for reading";
    throw runtime_error(what.str());
  }
  m_encoder = make_unique<MivEncoder>(m_stream);
}

void IvMetadataWriter::writeIvSequenceParams(const IvSequenceParams &ivSequenceParams) {
  m_encoder->writeIvSequenceParams(ivSequenceParams);
  m_frameRate = ivSequenceParams.frameRate;
}

void IvMetadataWriter::writeIvAccessUnitParams(const IvAccessUnitParams &ivAccessUnitParams,
                                               int intraPeriodFrameCount) {
  m_encoder->writeIvAccessUnitParams(ivAccessUnitParams, intraPeriodFrameCount);
  m_bytesWritten = m_stream.tellp();
}

void IvMetadataWriter::reportSummary(std::ostream &out) const {
  auto bitrate = 8e-3 * double(m_bytesWritten) / m_frameRate;
  out << "Total bitrate is " << bitrate << " kbps\n";
}
} // namespace TMIV::IO
