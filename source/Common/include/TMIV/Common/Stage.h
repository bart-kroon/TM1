/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#ifndef TMIV_COMMON_STAGE_H
#define TMIV_COMMON_STAGE_H

#include <type_traits>
#include <utility>
#include <vector>

namespace TMIV::Common {
// Interface of the input side of an encoding stage
template <typename In> class IStageSink {
public:
  // Pass by-value for thread safety and ease of reasoning
  static_assert(std::is_same_v<In, std::remove_cv_t<In>>);
  static_assert(std::is_same_v<In, std::remove_const_t<In>>);

  // Non-movable because that would invalidate the pointer in StageSource
  IStageSink() = default;
  IStageSink(const IStageSink &) = delete;
  IStageSink(IStageSink &&) = delete;
  auto operator=(const IStageSink &) -> IStageSink & = delete;
  auto operator=(IStageSink &&) -> IStageSink & = delete;
  virtual ~IStageSink() = default;

  // Input is pushed into this stage using this method
  virtual void encode(In unit) = 0;

  // End of input is signalled using this method
  virtual void flush() = 0;
};

// Output side of an encoding stage
template <typename Out> class StageSource {
public:
  // Pass by-value for thread safety and ease of reasoning
  static_assert(std::is_same_v<Out, std::remove_cv_t<Out>>);
  static_assert(std::is_same_v<Out, std::remove_const_t<Out>>);

  // Output is pushed out of this stage into the next one
  void connectTo(IStageSink<Out> &sink) { m_sink = &sink; }

  // If connected send a value to the next encoding stage
  void encode(Out unit) const {
    if (m_sink != nullptr) {
      m_sink->encode(std::move(unit));
    }
  }

  // If connected signal that the last value has been sent
  void flush() const {
    if (m_sink != nullptr) {
      m_sink->flush();
    }
  }

private:
  IStageSink<Out> *m_sink = nullptr;
};

// An encoding stage has an input stage interface and an output stage
template <typename In, typename Out> class Stage : public IStageSink<In> {
public:
  StageSource<Out> source;

  void flush() override { source.flush(); }
};

// An abstract stage that buffers units until a special unit arrives or the stage is flushed
template <typename In, typename Out> class BufferingStage : public Stage<In, Out> {
public:
  void encode(In unit) override {
    if (!m_buffer.empty() && isStart(unit)) {
      process(std::move(m_buffer));
      m_buffer = {};
    }
    m_buffer.push_back(std::move(unit));
  }

  void flush() override {
    if (!m_buffer.empty()) {
      process(std::move(m_buffer));
      m_buffer = {};
    }
    this->source.flush();
  }

protected:
  // Is this unit the start of a new segment of units?
  [[nodiscard]] virtual auto isStart(const In &unit) -> bool = 0;

  // Process all units in this segment, calling source.encode() multiple times
  virtual void process(std::vector<In> buffer) = 0;

private:
  std::vector<In> m_buffer;
};
} // namespace TMIV::Common

#endif
