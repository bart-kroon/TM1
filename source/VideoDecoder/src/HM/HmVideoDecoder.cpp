/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ITU/ISO/IEC
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

#include <TMIV/VideoDecoder/VideoDecoder.h>

#include <TMIV/Common/Decoder.h>
#include <TMIV/Common/Frame.h>

#include <TLibCommon/TComList.h>
#include <TLibCommon/TComPicYuv.h>
#include <TLibDecoder/AnnexBread.h>
#include <TLibDecoder/NALread.h>
#include <TLibDecoder/TDecTop.h>

#include <array>

namespace TMIV::VideoDecoder {
namespace {
// This implementation is based on TAppDec.cpp (HM 16.16) with all optional parameters locked to
// default values and without fields.
class HmVideoDecoder final : public Common::Decoder<std::string, Common::DecodedFrame> {
public:
  explicit HmVideoDecoder(Common::Source<std::string> source)
      : Common::Decoder<std::string, Common::DecodedFrame>{std::move(source)}
      , m_cTDecTop{new TDecTop} {
    m_cTDecTop->create();
    m_cTDecTop->init();
    m_cTDecTop->setDecodedPictureHashSEIEnabled(1);

    // set the last displayed POC correctly for skip forward.
    m_iPOCLastDisplay += m_iSkipFrame;
  }

protected:
  auto decodeSome() -> bool final {
    // inputBuffer serves to work around a design fault in the decoder, whereby the process
    // of reading a new slice that is the first slice of a new frame requires the TDecTop::decode()
    // method to be called again with the same nal unit.
    if (m_inputBuffer.empty()) {
      m_inputBuffer = takeNalUnit();
    }
    if (m_inputBuffer.empty()) {
      xFlushOutput();

      // NOTE(BK): It's either double delete or leaking memory. Easy to fix by putting a reference
      //           count in initROM/destroyROM, but the intention was not to modify HM 16.16.
      // deletePicBuffer();
      m_cTDecTop->destroy();

      return false;
    }

    // Copy the NAL unit payload into the HM representation of a NAL unit
    InputNALUnit nalu;
    nalu.getBitstream().getFifo().resize(m_inputBuffer.size());
    memcpy(nalu.getBitstream().getFifo().data(), m_inputBuffer.data(), m_inputBuffer.size());

    // call actual decoding function
    bool bNewPicture = false;
    read(nalu);
    bNewPicture = m_cTDecTop->decode(nalu, m_iSkipFrame, m_iPOCLastDisplay);
    if (!bNewPicture) {
      m_inputBuffer = takeNalUnit();
    }

    if ((bNewPicture || m_inputBuffer.empty() || nalu.m_nalUnitType == NAL_UNIT_EOS) &&
        !m_cTDecTop->getFirstSliceInSequence()) {
      if (!m_loopFiltered || !m_inputBuffer.empty()) {
        m_cTDecTop->executeLoopFilters(m_poc, m_pcListPic);
      }
      m_loopFiltered = (nalu.m_nalUnitType == NAL_UNIT_EOS);
      if (nalu.m_nalUnitType == NAL_UNIT_EOS) {
        m_cTDecTop->setFirstSliceInSequence(true);
      }
    } else if ((bNewPicture || m_inputBuffer.empty() || nalu.m_nalUnitType == NAL_UNIT_EOS) &&
               m_cTDecTop->getFirstSliceInSequence()) {
      m_cTDecTop->setFirstSliceInPicture(true);
    }

    if (m_pcListPic != nullptr) {
      if (m_outputBitDepth.front() == 0) {
        const auto &recon = m_pcListPic->front()->getPicSym()->getSPS().getBitDepths().recon;
        std::copy(std::cbegin(recon), std::cend(recon), std::begin(m_outputBitDepth));
      }

      if (bNewPicture) {
        xWriteOutput();
      }
      if ((bNewPicture || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA) &&
          m_cTDecTop->getNoOutputPriorPicsFlag()) {
        m_cTDecTop->checkNoOutputPriorPics(m_pcListPic);
        m_cTDecTop->setNoOutputPriorPicsFlag(false);
      }
      if (bNewPicture && (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
                          nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
                          nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP ||
                          nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
                          nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP)) {
        xFlushOutput();
      }
      if (nalu.m_nalUnitType == NAL_UNIT_EOS) {
        xWriteOutput();
        m_cTDecTop->setFirstSliceInPicture(false);
      }
      // write reconstruction to file -- for additional bumping as defined in C.5.2.3
      if (!bNewPicture && nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL_N &&
          nalu.m_nalUnitType <= NAL_UNIT_RESERVED_VCL31) {
        xWriteOutput();
      }
    }
    return true;
  }

private:
  auto takeNalUnit() -> std::string {
    if (!m_end) {
      if (auto unit = pull()) {
        return *unit;
      }
    }
    m_end = true;
    return {};
  }

  void xWriteOutput() {
    if (m_pcListPic->empty()) {
      return;
    }

    int32_t numPicsNotYetDisplayed = 0;
    int32_t dpbFullness = 0;
    const auto &activeSPS = m_pcListPic->front()->getPicSym()->getSPS();
    const auto maxNrSublayers = activeSPS.getMaxTLayers();
    const auto numReorderPicsHighestTid = activeSPS.getNumReorderPics(maxNrSublayers - 1);
    const auto maxDecPicBufferingHighestTid = activeSPS.getMaxDecPicBuffering(maxNrSublayers - 1);

    for (const auto *pcPic : *m_pcListPic) {
      if (pcPic->getOutputMark() && pcPic->getPOC() > m_iPOCLastDisplay) {
        numPicsNotYetDisplayed++;
        dpbFullness++;
      } else if (pcPic->getSlice(0)->isReferenced()) {
        dpbFullness++;
      }
    }

    for (auto *pcPic_ : *m_pcListPic) {
      if (pcPic_->getOutputMark() && pcPic_->getPOC() > m_iPOCLastDisplay &&
          (numPicsNotYetDisplayed > numReorderPicsHighestTid ||
           dpbFullness > maxDecPicBufferingHighestTid)) {
        numPicsNotYetDisplayed--;
        if (!pcPic_->getSlice(0)->isReferenced()) {
          dpbFullness--;
        }

        m_pcPic = pcPic_;
        xWritePicture();
      }
    }
  }

  void xFlushOutput() {
    if (m_pcListPic->empty()) {
      return;
    }

    for (auto *pcPic : *m_pcListPic) {
      if (pcPic->getOutputMark()) {
        m_pcPic = pcPic;
        xWritePicture();
      }
      if (pcPic != nullptr) {
        pcPic->destroy();
        delete pcPic; // NOLINT(cppcoreguidelines-owning-memory)
      }
    }

    m_pcListPic->clear();
    m_iPOCLastDisplay = -MAX_INT;
  }

  void xWritePicture() {
    xOutputPicture();

    // update POC of display order
    m_iPOCLastDisplay = m_pcPic->getPOC();

    // erase non-referenced comPic in the reference comPic list after display
    if (!m_pcPic->getSlice(0)->isReferenced() && m_pcPic->getReconMark()) {
      m_pcPic->setReconMark(false);

      // mark it should be extended later
      m_pcPic->getPicYuvRec()->setBorderExtension(false);
    }

    m_pcPic->setOutputMark(false);
  }

  void xOutputPicture() {
    auto *comPicYuv = m_pcPic->getPicYuvRec();
    PRECONDITION(comPicYuv != nullptr);

    auto outFrame = Common::Frame<>{};
    outFrame.setBitDepth(Common::at(m_outputBitDepth, int32_t{toChannelType(COMPONENT_Y)}));
    outFrame.getPlanes().resize(comPicYuv->getNumberValidComponents());

    for (const auto d : {COMPONENT_Y, COMPONENT_Cb, COMPONENT_Cr}) {
      if (d < comPicYuv->getNumberValidComponents()) {
        const auto planeBitDepth = Common::at(m_outputBitDepth, int32_t{toChannelType(d)});
        LIMITATION(planeBitDepth == outFrame.getBitDepth());

        const auto *row = comPicYuv->getAddr(d);
        const auto stride = comPicYuv->getStride(d);
        const auto width = comPicYuv->getWidth(d);
        const auto height = comPicYuv->getHeight(d);

        auto &outPlane = outFrame.getPlane(int32_t{d});
        outPlane.resize({static_cast<size_t>(height), static_cast<size_t>(width)});

        for (int32_t i = 0; i < height; ++i) {
          // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
          std::transform(row, row + width, outPlane.row_begin(i), [](const auto sample) {
            return Common::assertDownCast<Common::DefaultElement>(sample);
          });

          // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
          row += stride;
        }
      }
    }

    push({outFrame, m_pcPic->getSlice(0)->isIRAP()});
  }

  int32_t m_poc{};
  TComList<TComPic *> *m_pcListPic{nullptr};
  std::unique_ptr<TDecTop> m_cTDecTop{};
  int32_t m_iPOCLastDisplay{-MAX_INT};
  bool m_loopFiltered{};
  int32_t m_iSkipFrame{};
  std::string m_inputBuffer;
  bool m_end{};
  std::array<int32_t, MAX_NUM_CHANNEL_TYPE> m_outputBitDepth{};
  TComPic *m_pcPic{};
};
} // namespace

auto decodeHevcMain10(Common::Source<std::string> source) -> Common::Source<Common::DecodedFrame> {
  return [decoder = std::make_shared<HmVideoDecoder>(std::move(source))]() mutable {
    return (*decoder)();
  };
}
} // namespace TMIV::VideoDecoder
