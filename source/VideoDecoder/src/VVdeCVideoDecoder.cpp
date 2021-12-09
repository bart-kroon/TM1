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
struct VVdeCVideoDecoder::VVdeCContext {
  vvdecParams params{};
  vvdecAccessUnit *accessUnit{};
  vvdecDecoder *decoder{};
  vvdecFrame *frame{};
  bool flushing{};
};

namespace {
void loggingCallback(void * /* context */, int32_t /* level */, const char *fmt, va_list args) {
  vfprintf(stdout, fmt, args);
}
} // namespace

VVdeCVideoDecoder::VVdeCVideoDecoder(NalUnitSource source)
    : VideoDecoderBase{std::move(source)}, m_context{new VVdeCContext} {
  m_context->accessUnit = vvdec_accessUnit_alloc();
  VERIFY(m_context->accessUnit);

  vvdec_params_default(&m_context->params);
  m_context->params.logLevel = VVDEC_INFO;
  m_context->params.verifyPictureHash = true;
  m_context->params.parseThreads = 1;
  m_context->params.threads = 1;

  m_context->decoder = vvdec_decoder_open(&m_context->params);
  VERIFY(m_context->decoder);
  vvdec_set_logging_callback(m_context->decoder, loggingCallback);
  fmt::print("{}\n", vvdec_get_dec_information(m_context->decoder));
}

VVdeCVideoDecoder::~VVdeCVideoDecoder() {
  if (m_context->decoder != nullptr) {
    if (m_context->frame != nullptr) {
      const auto err = vvdec_frame_unref(m_context->decoder, m_context->frame);
      if (err != VVDEC_OK) {
        Common::assertionFailed(vvdec_get_error_msg(err), __FILE__, __LINE__);
      }
    }
    const auto err = vvdec_decoder_close(m_context->decoder);
    if (err != VVDEC_OK) {
      Common::assertionFailed(vvdec_get_error_msg(err), __FILE__, __LINE__);
    }
  }
  vvdec_accessUnit_free(m_context->accessUnit);
}

auto VVdeCVideoDecoder::decodeSome() -> bool {
  if (!m_context->flushing && (!takeAccessUnit() || !decodeFrame())) {
    m_context->flushing = true;
  }
  if (m_context->flushing && !flushFrame()) {
    return false;
  }
  if (m_context->frame != nullptr) {
    outputFrame();
    releaseFrame();
  }
  return true;
}

auto VVdeCVideoDecoder::takeAccessUnit() -> bool {
  auto blob = takeNalUnit();

  if (blob.empty()) {
    return false;
  }

  blob = "\0\0\1"s + blob; // Prefix a start code
  const auto payloadUsedSize = Common::downCast<int32_t>(blob.size());

  if (m_context->accessUnit->payloadSize < blob.size()) {
    vvdec_accessUnit_free_payload(m_context->accessUnit);
    vvdec_accessUnit_alloc_payload(m_context->accessUnit, payloadUsedSize);
  }
  memcpy(m_context->accessUnit->payload, blob.data(), blob.size());
  m_context->accessUnit->payloadUsedSize = payloadUsedSize;
  return true;
}

auto VVdeCVideoDecoder::decodeFrame() -> bool {
  const auto err = vvdec_decode(m_context->decoder, m_context->accessUnit, &m_context->frame);

  if (err == VVDEC_EOF) {
    return false;
  }
  if (err == VVDEC_TRY_AGAIN || err == VVDEC_OK) {
    return true;
  }
  throw std::runtime_error(fmt::format("Failed to decode VVC frame: {}", vvdec_get_error_msg(err)));
}

auto VVdeCVideoDecoder::flushFrame() -> bool {
  const auto err = vvdec_flush(m_context->decoder, &m_context->frame);

  if (err != VVDEC_OK && err != VVDEC_EOF) {
    throw std::runtime_error(vvdec_get_error_msg(err));
  }
  return m_context->frame != nullptr;
}

void VVdeCVideoDecoder::outputFrame() {
  LIMITATION(m_context->frame->frameFormat == VVDEC_FF_PROGRESSIVE);

  auto outFrame = Common::Frame<>{};
  outFrame.setBitDepth(m_context->frame->bitDepth);
  outFrame.getPlanes().resize(m_context->frame->numPlanes);

  for (uint32_t d = 0; d < m_context->frame->numPlanes; ++d) {
    const auto &inPlane = Common::at(m_context->frame->planes, d);
    auto &outPlane = outFrame.getPlane(d);

    outPlane.resize({inPlane.height, inPlane.width});

    Common::withElement(m_context->frame->bitDepth, [&](auto zero) {
      using Element = decltype(zero);

      for (uint32_t i = 0; i < inPlane.height; ++i) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        const auto *rowU8 = inPlane.ptr + ptrdiff_t{inPlane.stride} * ptrdiff_t{i};

        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        const auto *row = reinterpret_cast<const Element *>(rowU8);

        std::copy_n(row, inPlane.width, outPlane.row_begin(i));
      }
    });
  }

  return VideoDecoderBase::outputFrame(outFrame);
}

void VVdeCVideoDecoder::releaseFrame() {
  auto *frame = m_context->frame;
  m_context->frame = nullptr;

  const auto err = vvdec_frame_unref(m_context->decoder, frame);

  if (err != VVDEC_OK) {
    throw std::runtime_error(vvdec_get_error_msg(err));
  }
}
} // namespace TMIV::VideoDecoder