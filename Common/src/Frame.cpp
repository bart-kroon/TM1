/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
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

#include <TMIV/Common/Frame.h>

namespace TMIV::Common {

///////////////////////////////////////////////////////////
template <>
void convert(const Frame<YUV400P8> &inputFrame, Frame<YUV400P10> &outputFrame) {
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV400P8>::base_type v) {
        return (Frame<YUV400P10>::base_type)floor(1024.f * (float(v) / 256.f));
      });
}

template <>
void convert(const Frame<YUV400P8> &inputFrame, Frame<YUV400P16> &outputFrame) {
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV400P8>::base_type v) {
        return (Frame<YUV400P16>::base_type)floor(65536.f * (float(v) / 256.f));
      });
}

template <>
void convert(const Frame<YUV400P8> &inputFrame, Frame<YUV420P8> &outputFrame) {
  std::copy(inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
            outputFrame.getPlane(0).begin());
  std::fill(outputFrame.getPlane(1).begin(), outputFrame.getPlane(1).end(),
            uint8_t(128));
  std::fill(outputFrame.getPlane(2).begin(), outputFrame.getPlane(2).end(),
            uint8_t(128));
}

template <>
void convert(const Frame<YUV400P8> &inputFrame, Frame<YUV420P10> &outputFrame) {
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV400P8>::base_type v) {
        return (Frame<YUV420P10>::base_type)floor(1024.f * (float(v) / 256.f));
      });

  std::fill(outputFrame.getPlane(1).begin(), outputFrame.getPlane(1).end(),
            uint16_t(512));
  std::fill(outputFrame.getPlane(2).begin(), outputFrame.getPlane(2).end(),
            uint16_t(512));
}

template <>
void convert(const Frame<YUV400P8> &inputFrame, Frame<YUV420P16> &outputFrame) {
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV400P8>::base_type v) {
        return (Frame<YUV420P16>::base_type)floor(65536.f * (float(v) / 256.f));
      });

  std::fill(outputFrame.getPlane(1).begin(), outputFrame.getPlane(1).end(),
            uint16_t(32768));
  std::fill(outputFrame.getPlane(2).begin(), outputFrame.getPlane(2).end(),
            uint16_t(32768));
}

///////////////////////////////////////////////////////////
template <>
void convert(const Frame<YUV400P10> &inputFrame, Frame<YUV400P8> &outputFrame) {
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV400P10>::base_type v) {
        return (Frame<YUV400P8>::base_type)clamp(
            (float)floor(256.f * (float(v) / 1024.f) + 0.5f), 0.f, 255.f);
      });
}

template <>
void convert(const Frame<YUV400P10> &inputFrame,
             Frame<YUV400P16> &outputFrame) {
  std::transform(inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
                 outputFrame.getPlane(0).begin(),
                 [](Frame<YUV400P10>::base_type v) {
                   return (Frame<YUV400P16>::base_type)floor(
                       65536.f * (float(v) / 1024.f));
                 });
}

template <>
void convert(const Frame<YUV400P10> &inputFrame, Frame<YUV420P8> &outputFrame) {
    std::transform(
        inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
        outputFrame.getPlane(0).begin(), [](Frame<YUV400P10>::base_type v) {
          return (Frame<YUV420P8>::base_type)clamp(
              (float)floor(256.f * (float(v) / 1024.f) + 0.5f), 0.f, 255.f);
        });

  std::fill(outputFrame.getPlane(1).begin(), outputFrame.getPlane(1).end(),
            uint8_t(128));
  std::fill(outputFrame.getPlane(2).begin(), outputFrame.getPlane(2).end(),
            uint8_t(128));
}

template <>
void convert(const Frame<YUV400P10> &inputFrame, Frame<YUV420P10> &outputFrame) {
  std::copy(inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
            outputFrame.getPlane(0).begin());

  std::fill(outputFrame.getPlane(1).begin(), outputFrame.getPlane(1).end(),
            uint16_t(512));
  std::fill(outputFrame.getPlane(2).begin(), outputFrame.getPlane(2).end(),
            uint16_t(512));
}

template <>
void convert(const Frame<YUV400P10> &inputFrame,
             Frame<YUV420P16> &outputFrame) {
    std::transform(inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
                   outputFrame.getPlane(0).begin(),
                   [](Frame<YUV420P10>::base_type v) {
                     return (Frame<YUV400P16>::base_type)floor(
                         65536.f * (float(v) / 1024.f));
                   });

  std::fill(outputFrame.getPlane(1).begin(), outputFrame.getPlane(1).end(),
            uint16_t(32768));
  std::fill(outputFrame.getPlane(2).begin(), outputFrame.getPlane(2).end(),
            uint16_t(32768));
}

///////////////////////////////////////////////////////////
template <>
void convert(const Frame<YUV400P16> &inputFrame, Frame<YUV400P8> &outputFrame) {
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV400P16>::base_type v) {
        return (Frame<YUV400P8>::base_type)clamp(
            (float)floor(256.f * (float(v) / 65536.f) + 0.5f), 0.f, 255.f);
      });
}

template <>
void convert(const Frame<YUV400P16> &inputFrame,
             Frame<YUV400P10> &outputFrame) {
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV400P16>::base_type v) {
        return (Frame<YUV400P10>::base_type)clamp(
            (float)floor(1024.f * (float(v) / 65536.f) + 0.5f), 0.f, 1023.f);
      });
}

template <>
void convert(const Frame<YUV400P16> &inputFrame, Frame<YUV420P8> &outputFrame) {
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV400P16>::base_type v) {
        return (Frame<YUV420P8>::base_type)clamp(
            (float)floor(256.f * (float(v) / 65536.f) + 0.5f), 0.f, 255.f);
      });

  std::fill(outputFrame.getPlane(1).begin(), outputFrame.getPlane(1).end(),
            uint8_t(128));
  std::fill(outputFrame.getPlane(2).begin(), outputFrame.getPlane(2).end(),
            uint8_t(128));
}

template <>
void convert(const Frame<YUV400P16> &inputFrame,
             Frame<YUV420P10> &outputFrame) {
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV400P16>::base_type v) {
        return (Frame<YUV420P10>::base_type)clamp(
            (float)floor(1024.f * (float(v) / 65536.f) + 0.5f), 0.f, 1023.f);
      });

  std::fill(outputFrame.getPlane(1).begin(), outputFrame.getPlane(1).end(),
            uint16_t(512));
  std::fill(outputFrame.getPlane(2).begin(), outputFrame.getPlane(2).end(),
            uint16_t(512));
}

template <>
void convert(const Frame<YUV400P16> &inputFrame,
             Frame<YUV420P16> &outputFrame) {
  std::copy(inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
            outputFrame.getPlane(0).begin());
  std::fill(outputFrame.getPlane(1).begin(), outputFrame.getPlane(1).end(),
            uint16_t(32768));
  std::fill(outputFrame.getPlane(2).begin(), outputFrame.getPlane(2).end(),
            uint16_t(32768));
}

///////////////////////////////////////////////////////////
template <>
void convert(const Frame<YUV420P8> &inputFrame, Frame<YUV400P8> &outputFrame) {
  std::copy(inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
            outputFrame.getPlane(0).begin());
}

template <>
void convert(const Frame<YUV420P8> &inputFrame, Frame<YUV400P10> &outputFrame) {
    std::transform(inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
                   outputFrame.getPlane(0).begin(),
                   [](Frame<YUV420P8>::base_type v) {
                     return (Frame<YUV400P10>::base_type)floor(
                         1024.f * (float(v) / 256.f));
                   });
}

template <>
void convert(const Frame<YUV420P8> &inputFrame, Frame<YUV400P16> &outputFrame) {
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV400P8>::base_type v) {
        return (Frame<YUV400P16>::base_type)floor(65536.f * (float(v) / 256.f));
      });
}

template <>
void convert(const Frame<YUV420P8> &inputFrame, Frame<YUV420P10> &outputFrame) {
  for (int i = 0; i < 3; i++) {
    std::transform(inputFrame.getPlane(i).begin(), inputFrame.getPlane(i).end(),
                   outputFrame.getPlane(i).begin(),
                   [](Frame<YUV420P8>::base_type v) {
                     return (Frame<YUV420P10>::base_type)floor(
                         1024.f * (float(v) / 256.f));
                   });
  }
}

template <>
void convert(const Frame<YUV420P8> &inputFrame, Frame<YUV420P16> &outputFrame) {
  for (int i = 0; i < 3; i++) {
    std::transform(inputFrame.getPlane(i).begin(), inputFrame.getPlane(i).end(),
                   outputFrame.getPlane(i).begin(),
                   [](Frame<YUV420P8>::base_type v) {
                     return (Frame<YUV420P16>::base_type)floor(
                         65536.f * (float(v) / 256.f));
                   });
  }
}

///////////////////////////////////////////////////////////
template <>
void convert(const Frame<YUV420P10> &inputFrame, Frame<YUV400P8> &outputFrame) {
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV420P10>::base_type v) {
        return (Frame<YUV400P8>::base_type)clamp(
            (float)floor(256.f * (float(v) / 1024.f) + 0.5f), 0.f, 255.f);
      });
}

template <>
void convert(const Frame<YUV420P10> &inputFrame, Frame<YUV400P10> &outputFrame) {
  std::copy(inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
            outputFrame.getPlane(0).begin());
}

template <>
void convert(const Frame<YUV420P10> &inputFrame,
             Frame<YUV400P16> &outputFrame) {
  std::transform(inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
                 outputFrame.getPlane(0).begin(),
                 [](Frame<YUV420P10>::base_type v) {
                   return (Frame<YUV400P16>::base_type)floor(
                       65536.f * (float(v) / 1024.f));
                 });
}

template <>
void convert(const Frame<YUV420P10> &inputFrame, Frame<YUV420P8> &outputFrame) {
  for (int i = 0; i < 3; i++) {
    std::transform(
        inputFrame.getPlane(i).begin(), inputFrame.getPlane(i).end(),
        outputFrame.getPlane(i).begin(), [](Frame<YUV420P10>::base_type v) {
          return (Frame<YUV420P8>::base_type)clamp(
              (float)floor(256.f * (float(v) / 1024.f) + 0.5f), 0.f, 255.f);
        });
  }
}

template <>
void convert(const Frame<YUV420P10> &inputFrame,
             Frame<YUV420P16> &outputFrame) {
  for (int i = 0; i < 3; i++) {
    std::transform(inputFrame.getPlane(i).begin(), inputFrame.getPlane(i).end(),
                   outputFrame.getPlane(i).begin(),
                   [](Frame<YUV420P10>::base_type v) {
                     return (Frame<YUV400P16>::base_type)floor(
                         65536.f * (float(v) / 1024.f));
                   });
  }
}

///////////////////////////////////////////////////////////
template <>
void convert(const Frame<YUV420P16> &inputFrame, Frame<YUV400P8> &outputFrame) {
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV420P16>::base_type v) {
        return (Frame<YUV400P8>::base_type)clamp(
            (float)floor(256.f * (float(v) / 65536.f) + 0.5f), 0.f, 255.f);
      });
}

template <>
void convert(const Frame<YUV420P16> &inputFrame,
             Frame<YUV400P10> &outputFrame) {
    std::transform(
        inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
        outputFrame.getPlane(0).begin(), [](Frame<YUV420P16>::base_type v) {
          return (Frame<YUV420P10>::base_type)clamp(
              (float)floor(1024.f * (float(v) / 65536.f) + 0.5f), 0.f, 1023.f);
        });
}

template <>
void convert(const Frame<YUV420P16> &inputFrame,
             Frame<YUV400P16> &outputFrame) {
  std::copy(inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
            outputFrame.getPlane(0).begin());
}

template <>
void convert(const Frame<YUV420P16> &inputFrame, Frame<YUV420P8> &outputFrame) {
  for (int i = 0; i < 3; i++) {
    std::transform(
        inputFrame.getPlane(i).begin(), inputFrame.getPlane(i).end(),
        outputFrame.getPlane(i).begin(), [](Frame<YUV420P16>::base_type v) {
          return (Frame<YUV420P8>::base_type)clamp(
              (float)floor(256.f * (float(v) / 65536.f) + 0.5f), 0.f, 65535.f);
        });
  }
}

template <>
void convert(const Frame<YUV420P16> &inputFrame,
             Frame<YUV420P10> &outputFrame) {
  for (int i = 0; i < 3; i++) {
    std::transform(
        inputFrame.getPlane(i).begin(), inputFrame.getPlane(i).end(),
        outputFrame.getPlane(i).begin(), [](Frame<YUV420P16>::base_type v) {
          return (Frame<YUV420P10>::base_type)clamp(
              (float)floor(1024.f * (float(v) / 65536.f) + 0.5f), 0.f, 1023.f);
        });
  }
}
} // namespace TMIV::Common
