/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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

#include <TMIV/VideoDecoder/HmVideoDecoder.h>

#include <TLibCommon/TComList.h>
#include <TLibCommon/TComPicYuv.h>
#include <TLibDecoder/AnnexBread.h>
#include <TLibDecoder/NALread.h>
#include <TLibDecoder/TDecTop.h>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::VideoDecoder {
class HmPicture : public IDecodedPicture {
public:
  HmPicture(const TComPic &picture) : m_picture{picture} {}
  HmPicture(const HmPicture &other) = delete;
  HmPicture(HmPicture &&other) = delete;
  HmPicture &operator=(const HmPicture &other) = delete;
  HmPicture &operator=(HmPicture &&other) = delete;
  ~HmPicture() = default;

private:
  const TComPic &m_picture;
};

class HmVideoDecoder::Impl {
public:
  void decode(std::istream &stream) {
    int poc{};
    TComList<TComPic *> *pcListPic = nullptr;

    InputByteStream bytestream(stream);

    // create & initialize internal classes
    m_cTDecTop.create();
    m_cTDecTop.init();
    m_cTDecTop.setDecodedPictureHashSEIEnabled(1);

    // set the last displayed POC correctly for skip forward.
    m_iPOCLastDisplay += m_iSkipFrame;

    // reconstruction file not yet opened. (must be performed after SPS is seen)
    bool loopFiltered = false;

    // main decoder loop
    while (!!stream) {
      /* location serves to work around a design fault in the decoder, whereby
       * the process of reading a new slice that is the first slice of a new frame
       * requires the TDecTop::decode() method to be called again with the same
       * nal unit. */
      streampos location = stream.tellg();
      AnnexBStats stats = AnnexBStats();

      InputNALUnit nalu;
      byteStreamNALUnit(bytestream, nalu.getBitstream().getFifo(), stats);

      // call actual decoding function
      bool bNewPicture = false;
      if (nalu.getBitstream().getFifo().empty()) {
        fprintf(stderr, "Warning: Attempt to decode an empty NAL unit\n");
      } else {
        read(nalu);
        bNewPicture = m_cTDecTop.decode(nalu, m_iSkipFrame, m_iPOCLastDisplay);
        if (bNewPicture) {
          stream.clear();
          stream.seekg(location - streamoff(3));
          bytestream.reset();
        }
      }

      if ((bNewPicture || !stream || nalu.m_nalUnitType == NAL_UNIT_EOS) &&
          !m_cTDecTop.getFirstSliceInSequence()) {
        if (!loopFiltered || stream) {
          m_cTDecTop.executeLoopFilters(poc, pcListPic);
        }
        loopFiltered = (nalu.m_nalUnitType == NAL_UNIT_EOS);
        if (nalu.m_nalUnitType == NAL_UNIT_EOS) {
          m_cTDecTop.setFirstSliceInSequence(true);
        }
      } else if ((bNewPicture || !stream || nalu.m_nalUnitType == NAL_UNIT_EOS) &&
                 m_cTDecTop.getFirstSliceInSequence()) {
        m_cTDecTop.setFirstSliceInPicture(true);
      }

      if (pcListPic) {
        if (bNewPicture) {
          xWriteOutput(*pcListPic, nalu.m_temporalId);
        }
        if ((bNewPicture || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA) &&
            m_cTDecTop.getNoOutputPriorPicsFlag()) {
          m_cTDecTop.checkNoOutputPriorPics(pcListPic);
          m_cTDecTop.setNoOutputPriorPicsFlag(false);
        }
        if (bNewPicture && (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
                            nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
                            nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP ||
                            nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
                            nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP)) {
          xFlushOutput(*pcListPic);
        }
        if (nalu.m_nalUnitType == NAL_UNIT_EOS) {
          xWriteOutput(*pcListPic, nalu.m_temporalId);
          m_cTDecTop.setFirstSliceInPicture(false);
        }
        // write reconstruction to file -- for additional bumping as defined in C.5.2.3
        if (!bNewPicture && nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL_N &&
            nalu.m_nalUnitType <= NAL_UNIT_RESERVED_VCL31) {
          xWriteOutput(*pcListPic, nalu.m_temporalId);
        }
      }
    }

    xFlushOutput(*pcListPic);
    m_cTDecTop.deletePicBuffer();
    m_cTDecTop.destroy();
  }

  void addListener(Listener listener) { m_listeners.push_back(move(listener)); }

private:
  void xWriteOutput(TComList<TComPic *> &pcListPic, unsigned tId) {
    if (pcListPic.empty()) {
      return;
    }

    int numPicsNotYetDisplayed = 0;
    int dpbFullness = 0;
    const auto &activeSPS = pcListPic.front()->getPicSym()->getSPS();
    const auto maxNrSublayers = activeSPS.getMaxTLayers();
    const auto numReorderPicsHighestTid = activeSPS.getNumReorderPics(maxNrSublayers - 1);
    const auto maxDecPicBufferingHighestTid = activeSPS.getMaxDecPicBuffering(maxNrSublayers - 1);

    for (const auto *pcPic : pcListPic) {
      if (pcPic->getOutputMark() && pcPic->getPOC() > m_iPOCLastDisplay) {
        numPicsNotYetDisplayed++;
        dpbFullness++;
      } else if (pcPic->getSlice(0)->isReferenced()) {
        dpbFullness++;
      }
    }

    for (auto *pcPic : pcListPic) {
      if (pcPic->getOutputMark() && pcPic->getPOC() > m_iPOCLastDisplay &&
          (numPicsNotYetDisplayed > numReorderPicsHighestTid ||
           dpbFullness > maxDecPicBufferingHighestTid)) {
        numPicsNotYetDisplayed--;
        if (pcPic->getSlice(0)->isReferenced() == false) {
          dpbFullness--;
        }

        xWritePicture(*pcPic);
      }
    }
  }

  void xFlushOutput(TComList<TComPic *> &pcListPic) {
    if (pcListPic.empty()) {
      return;
    }

    for (auto *pcPic : pcListPic) {
      if (pcPic->getOutputMark()) {
        xWritePicture(*pcPic);
      }
      if (pcPic != nullptr) {
        pcPic->destroy();
        delete pcPic;
      }
    }

    pcListPic.clear();
    m_iPOCLastDisplay = -MAX_INT;
  }

  void xWritePicture(TComPic &pcPic) {
    // Wrap TComPic to provide IPicture interface
    const auto picture = HmPicture{pcPic};

    // Invoke all listeners
    for (const auto &listener : m_listeners) {
      listener(picture);
    }

    // update POC of display order
    m_iPOCLastDisplay = pcPic.getPOC();

    // erase non-referenced picture in the reference picture list after display
    if (!pcPic.getSlice(0)->isReferenced() && pcPic.getReconMark()) {
      pcPic.setReconMark(false);

      // mark it should be extended later
      pcPic.getPicYuvRec()->setBorderExtension(false);
    }

    pcPic.setOutputMark(false);
  }

  TDecTop m_cTDecTop{};

  vector<Listener> m_listeners;

  int m_iPOCLastDisplay{-MAX_INT};
  int m_iSkipFrame{};
}; // namespace TMIV::VideoDecoder

HmVideoDecoder::HmVideoDecoder() : m_impl{new Impl{}} {}

HmVideoDecoder::~HmVideoDecoder() = default;

void HmVideoDecoder::decode(std::istream &stream) { m_impl->decode(stream); }

void HmVideoDecoder::addListener(Listener listener) { return m_impl->addListener(move(listener)); }
} // namespace TMIV::VideoDecoder
