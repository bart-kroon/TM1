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

#include <TMIV/Common/Bytestream.h>
#include <TMIV/Common/Decoder.h>
#include <TMIV/Common/Formatters.h>
#include <TMIV/Common/Frame.h>
#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/Common/verify.h>

#include <vvdec/vvdec.h>

#include <list>
#include <mutex>

using namespace std::string_literals;

namespace TMIV::VideoDecoder {
namespace {
class VVdeCVideoDecoder final : public Common::Decoder<std::string, Common::DecodedFrame> {
public:
  explicit VVdeCVideoDecoder(Common::Source<std::string> source)
      : Common::Decoder<std::string, Common::DecodedFrame>{std::move(source)} {
    m_accessUnit = vvdec_accessUnit_alloc();
    VERIFY(m_accessUnit);

    vvdecParams params;
    vvdec_params_default(&params);
    params.logLevel = VVDEC_INFO;
    params.verifyPictureHash = true;
    params.parseThreads = 1;
    params.threads = 1;

    m_decoder = vvdec_decoder_open(&params);
    VERIFY(m_decoder);

    verify(vvdec_set_logging_callback(m_decoder, loggingCallback), __FILE__, __LINE__);

    Common::logInfo(vvdec_get_dec_information(m_decoder));
  }

  VVdeCVideoDecoder(const VVdeCVideoDecoder &) = delete;
  VVdeCVideoDecoder(VVdeCVideoDecoder &&) = delete;
  auto operator=(const VVdeCVideoDecoder &) -> VVdeCVideoDecoder & = delete;
  auto operator=(VVdeCVideoDecoder &&) -> VVdeCVideoDecoder & = delete;

  ~VVdeCVideoDecoder() final {
    if (m_decoder != nullptr) {
      if (m_frame != nullptr) {
        verify(vvdec_frame_unref(m_decoder, m_frame), __FILE__, __LINE__);
      }
      verify(vvdec_decoder_close(m_decoder), __FILE__, __LINE__);
    }
    vvdec_accessUnit_free(m_accessUnit);
  }

protected:
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
  [[nodiscard]] static auto translateLogLevel(int32_t level) -> Common::LogLevel {
    switch (level) {
    case VVDEC_ERROR:
      return Common::LogLevel::error;
    case VVDEC_WARNING:
      return Common::LogLevel::warning;
    case VVDEC_INFO:
    case VVDEC_NOTICE:
      return Common::LogLevel::info;
    case VVDEC_VERBOSE:
      return Common::LogLevel::verbose;
    default:
      return Common::LogLevel::debug;
    }
  }

  struct LineBuffer {
    LineBuffer(void *context) : context{context} {}

    void *context{};
    Common::LogLevel level{Common::LogLevel::debug};
    std::vector<char> buffer;

    void append(Common::LogLevel level_, char character) {
      level = std::min(level, level_);

      if (character == '\n') {
        const auto what = std::string_view{buffer.data(), buffer.size()};
        Common::logMessage(level, "[VVdeC @ {}] {}", context, what);
        buffer.clear();
        level = Common::LogLevel::debug;
      } else {
        buffer.push_back(character);
      }
    }
  };

  [[nodiscard]] static auto lineBuffer(void *context) -> LineBuffer & {
    static std::list<LineBuffer> buffers; // stable addressing
    static std::mutex mutex;

    std::unique_lock<std::mutex> lock{mutex};

    for (auto &buffer : buffers) {
      if (buffer.context == context) {
        return buffer;
      }
    }

    return buffers.emplace_back(context);
  }

  static void loggingCallback(void *context, int32_t level, const char *fmt, va_list args) {
    auto &lineBuffer_ = lineBuffer(context);
    const auto level_ = translateLogLevel(level);
    auto charBuffer = std::array<char, 0x1000>{};
    const auto charCount = vsnprintf(charBuffer.data(), charBuffer.size(), fmt, args);

    for (int32_t i = 0; i < charCount; ++i) {
      lineBuffer_.append(level_, Common::at(charBuffer, i));
    }
  }

  static void verify(int32_t err, const char *file, int32_t line) {
    if (err != VVDEC_OK) {
      Common::assertionFailed(vvdec_get_error_msg(err), file, line);
    }
  }

  auto takeAccessUnit() -> bool {
    if (auto unit = pull()) {
      *unit = "\0\0\1"s + *unit; // Prefix a start code
      const auto payloadUsedSize = Common::downCast<int32_t>(unit->size());

      if (static_cast<size_t>(m_accessUnit->payloadSize) < unit->size()) {
        vvdec_accessUnit_free_payload(m_accessUnit);
        vvdec_accessUnit_alloc_payload(m_accessUnit, payloadUsedSize);
      }
      memcpy(m_accessUnit->payload, unit->data(), unit->size());
      m_accessUnit->payloadUsedSize = payloadUsedSize;
      return true;
    }
    return false;
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
      throw std::runtime_error(
          fmt::format("Failed to flush VVC frame: {}", vvdec_get_error_msg(err)));
    }
    return m_frame != nullptr;
  }

  void outputFrame() {
    LIMITATION(m_frame->frameFormat == VVDEC_FF_PROGRESSIVE);

    auto outFrame = Common::Frame<>{};
    outFrame.setBitDepth(m_frame->bitDepth);
    outFrame.getPlanes().resize(m_frame->numPlanes);

    for (uint32_t d = 0; d < m_frame->numPlanes; ++d) {
      const auto &inPlane = Common::at(m_frame->planes, d);
      auto &outPlane = outFrame.getPlane(d);

      outPlane.resize({inPlane.height, inPlane.width});

      Common::withElement(m_frame->bitDepth, [&](auto zero) {
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

    VERIFY(m_frame->picAttributes != nullptr);
    const auto irap = VVC_NAL_UNIT_CODED_SLICE_IDR_W_RADL <= m_frame->picAttributes->nalType &&
                      m_frame->picAttributes->nalType <= VVC_NAL_UNIT_RESERVED_IRAP_VCL_12;
    return push({outFrame, irap});
  }

  void releaseFrame() {
    auto *frame = std::exchange(m_frame, nullptr);
    verify(vvdec_frame_unref(m_decoder, frame), __FILE__, __LINE__);
  }

  vvdecAccessUnit *m_accessUnit{};
  vvdecDecoder *m_decoder{};
  vvdecFrame *m_frame{};
  bool m_flushing{};
};
} // namespace

auto decodeVvcMain10(Common::Source<std::string> source) -> Common::Source<Common::DecodedFrame> {
  return [decoder = std::make_shared<VVdeCVideoDecoder>(std::move(source))]() mutable {
    return (*decoder)();
  };
}
} // namespace TMIV::VideoDecoder
