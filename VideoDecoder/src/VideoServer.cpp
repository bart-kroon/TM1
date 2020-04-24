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

#include <TMIV/VideoDecoder/VideoServer.h>

#include <condition_variable>
#include <iostream>
#include <sstream>
#include <thread>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::VideoDecoder {
class VideoServer::Impl {
public:
  Impl(unique_ptr<IVideoDecoder> decoder, string bitstream)
      : m_decoder{move(decoder)}, m_bitstream{move(bitstream)}, m_thread{[this]() { decode(); }} {}
  Impl(const Impl &) = delete;
  Impl(Impl &&) = delete;
  Impl &operator=(const Impl &) = delete;
  Impl &operator=(Impl &&) = delete;

  ~Impl() {
    m_requestStop = true;
    m_thread.join();
  }

  auto getFrame() -> unique_ptr<AnyFrame> {
    unique_lock<mutex> lock{m_mutex};
    m_cv.wait(lock, [this] { return m_frame || m_hasStopped; });
    if (m_hasStopped) {
      return {};
    }
    auto frame = unique_ptr<AnyFrame>{};
    swap(frame, m_frame);
    return frame;
  }

private:
  class Stop {};

  void decode() {
    try {
      m_decoder->addFrameListener([this](const AnyFrame &picture) { return listen(picture); });
      m_decoder->decode(m_bitstream);
      m_hasStopped = true;
    } catch (Stop & /* unused */) {
      m_hasStopped = true;
    } catch (exception &e) {
      cout << "Exception in video decoder: " << e.what() << '\n';
      abort();
    }
  }

  void listen(const AnyFrame &picture) {
    unique_lock<mutex> lock{m_mutex};
    m_cv.wait(lock, [this] { return !m_frame || m_requestStop; });
    if (m_requestStop) {
      throw Stop{};
    }
    m_frame.reset(new AnyFrame{picture});
  }

  unique_ptr<IVideoDecoder> m_decoder;
  istringstream m_bitstream;
  thread m_thread;
  mutex m_mutex;
  condition_variable m_cv;
  bool m_requestStop{};
  bool m_hasStopped{};
  unique_ptr<AnyFrame> m_frame{};
};

VideoServer::VideoServer(std::unique_ptr<IVideoDecoder> decoder, string bitstream)
    : m_impl{new Impl{move(decoder), move(bitstream)}} {}

VideoServer::~VideoServer() = default;

auto VideoServer::getFrame() -> unique_ptr<AnyFrame> { return m_impl->getFrame(); }
} // namespace TMIV::VideoDecoder
