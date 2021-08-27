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

#include <TMIV/VideoDecoder/VVdeCVideoDecoder.h>

#include <TMIV/Common/Bytestream.h>
#include <TMIV/Common/verify.h>
#include <TMIV/VideoDecoder/VideoDecoderBase.h>

#include <vvdec/vvdec.h>

#include <fmt/format.h>

using namespace std::string_literals;

namespace TMIV::VideoDecoder {
class VVdeCVideoDecoder::Impl : public VideoDecoderBase {
public:
  Impl(NalUnitSource source) : VideoDecoderBase{std::move(source)} {
    m_accessUnit = vvdec_accessUnit_alloc();
    VERIFY(m_accessUnit);

    vvdec_params_default(&m_params);
    m_params.logLevel = VVDEC_INFO;
    m_params.verifyPictureHash = true;
    m_params.parseThreads = 1;
    m_params.threads = 1;

    m_decoder = vvdec_decoder_open(&m_params);
    VERIFY(m_decoder);
    vvdec_set_logging_callback(m_decoder, &loggingCallback);
    fmt::print("{}\n", vvdec_get_dec_information(m_decoder));
  }

  Impl(const Impl &) = delete;
  Impl(Impl &&) = delete;
  auto operator=(const Impl &) -> Impl & = delete;
  auto operator=(Impl &&) -> Impl & = delete;

  ~Impl() final {
    if (m_decoder != nullptr) {
      if (m_frame != nullptr) {
        const auto err = vvdec_frame_unref(m_decoder, m_frame);
        if (err != VVDEC_OK) {
          Common::assertionFailed(vvdec_get_error_msg(err), __FILE__, __LINE__);
        }
      }
      const auto err = vvdec_decoder_close(m_decoder);
      if (err != VVDEC_OK) {
        Common::assertionFailed(vvdec_get_error_msg(err), __FILE__, __LINE__);
      }
    }
    vvdec_accessUnit_free(m_accessUnit);
  }

  auto decodeSome() -> bool final {
    if (!m_flushing && (!takeAccessUnit() || !decodeFrame())) {
      m_flushing = true;
    }
    if (m_flushing && !flushFrame()) {
      return false;
    }
    if (m_frame != nullptr) {
      outputFrame();
      releaseFrame();
    }
    return true;
  }

private:
  auto takeAccessUnit() -> bool {
    auto blob = takeNalUnit();

    if (blob.empty()) {
      return false;
    }

    blob = "\0\0\1"s + blob; // Prefix a start code
    const auto payloadUsedSize = Common::downCast<int32_t>(blob.size());

    if (m_accessUnit->payloadSize < blob.size()) {
      vvdec_accessUnit_free_payload(m_accessUnit);
      vvdec_accessUnit_alloc_payload(m_accessUnit, payloadUsedSize);
    }
    memcpy(m_accessUnit->payload, blob.data(), blob.size());
    m_accessUnit->payloadUsedSize = payloadUsedSize;
    return true;
  }

  auto decodeFrame() -> bool {
    const auto err = vvdec_decode(m_decoder, m_accessUnit, &m_frame);

    if (err == VVDEC_EOF) {
      return false;
    }
    if (err == VVDEC_TRY_AGAIN || err == VVDEC_OK) {
      return true;
    }
    throw std::runtime_error(
        fmt::format("Failed to decode VVC frame: {}", vvdec_get_error_msg(err)));
  }

  auto flushFrame() -> bool {
    const auto err = vvdec_flush(m_decoder, &m_frame);

    if (err != VVDEC_OK && err != VVDEC_EOF) {
      throw std::runtime_error(vvdec_get_error_msg(err));
    }
    return m_frame != nullptr;
  }

  void outputFrame() {
    LIMITATION(m_frame->frameFormat == VVDEC_FF_PROGRESSIVE);

    auto anyFrame = std::make_unique<Common::AnyFrame>();

    for (uint32_t i = 0; i < m_frame->numPlanes; ++i) {
      Common::at(anyFrame->bitdepth, i) = m_frame->bitDepth;

      const auto &inPlane = Common::at(m_frame->planes, i);
      auto &outPlane = Common::at(anyFrame->planes, i);
      outPlane.resize(inPlane.height, inPlane.width);

      for (uint32_t j = 0; j < inPlane.height; ++j) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        const auto *inRowU8 = inPlane.ptr + j * inPlane.stride;

        if (m_frame->bitDepth <= 8) {
          std::copy_n(inRowU8, inPlane.width, outPlane.row_begin(j));
        } else if (m_frame->bitDepth <= 16) {
          // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
          const auto *inRowU16 = reinterpret_cast<const uint16_t *>(inRowU8);
          std::copy_n(inRowU16, inPlane.width, outPlane.row_begin(j));
        } else {
          UNREACHABLE;
        }
      }
    }
    VideoDecoderBase::outputFrame(std::move(anyFrame));
  }

  void releaseFrame() {
    auto *frame = m_frame;
    m_frame = nullptr;

    const auto err = vvdec_frame_unref(m_decoder, frame);

    if (err != VVDEC_OK) {
      throw std::runtime_error(vvdec_get_error_msg(err));
    }
  }

  static void loggingCallback(void * /* context */, int32_t /* level */, const char *fmt,
                              va_list args) {
    vfprintf(stdout, fmt, args);
  }

  vvdecParams m_params{};
  vvdecAccessUnit *m_accessUnit{nullptr};
  vvdecDecoder *m_decoder{nullptr};
  vvdecFrame *m_frame{nullptr};
  bool m_flushing{};
};

VVdeCVideoDecoder::VVdeCVideoDecoder(NalUnitSource source) : m_impl{new Impl{std::move(source)}} {}

VVdeCVideoDecoder::~VVdeCVideoDecoder() = default;

auto VVdeCVideoDecoder::getFrame() -> std::unique_ptr<Common::AnyFrame> {
  return m_impl->getFrame();
}
} // namespace TMIV::VideoDecoder