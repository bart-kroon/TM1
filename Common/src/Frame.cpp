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
#include <algorithm>
#include <cassert>
#include <sstream>

namespace TMIV::Common {

///////////////////////////////////////////////////////////
template <>
void convert(const Frame<YUV400P8> &inputFrame, Frame<YUV400P10> &outputFrame) {
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV400P8>::base_type v) {
        return (Frame<YUV400P10>::base_type)floor(1024.f * (float(v) / 256.f));
      });
}

template <>
void convert(const Frame<YUV400P8> &inputFrame, Frame<YUV400P16> &outputFrame) {
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV400P8>::base_type v) {
        return (Frame<YUV400P16>::base_type)floor(65536.f * (float(v) / 256.f));
      });
}

template <>
void convert(const Frame<YUV400P8> &inputFrame, Frame<YUV420P8> &outputFrame) {
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
  std::copy(inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
            outputFrame.getPlane(0).begin());
  std::fill(outputFrame.getPlane(1).begin(), outputFrame.getPlane(1).end(),
            uint8_t(128));
  std::fill(outputFrame.getPlane(2).begin(), outputFrame.getPlane(2).end(),
            uint8_t(128));
}

template <>
void convert(const Frame<YUV400P8> &inputFrame, Frame<YUV420P10> &outputFrame) {
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
void convert(const Frame<YUV400P10> &inputFrame,
             Frame<YUV420P10> &outputFrame) {
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV400P16>::base_type v) {
        return (Frame<YUV400P10>::base_type)clamp(
            (float)floor(1024.f * (float(v) / 65536.f) + 0.5f), 0.f, 1023.f);
      });
}

template <>
void convert(const Frame<YUV400P16> &inputFrame, Frame<YUV420P8> &outputFrame) {
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
  std::copy(inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
            outputFrame.getPlane(0).begin());
}

template <>
void convert(const Frame<YUV420P8> &inputFrame, Frame<YUV400P10> &outputFrame) {
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV420P8>::base_type v) {
        return (Frame<YUV400P10>::base_type)floor(1024.f * (float(v) / 256.f));
      });
}

template <>
void convert(const Frame<YUV420P8> &inputFrame, Frame<YUV400P16> &outputFrame) {
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
  std::transform(
      inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
      outputFrame.getPlane(0).begin(), [](Frame<YUV420P10>::base_type v) {
        return (Frame<YUV400P8>::base_type)clamp(
            (float)floor(256.f * (float(v) / 1024.f) + 0.5f), 0.f, 255.f);
      });
}

template <>
void convert(const Frame<YUV420P10> &inputFrame,
             Frame<YUV400P10> &outputFrame) {
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
  std::copy(inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
            outputFrame.getPlane(0).begin());
}

template <>
void convert(const Frame<YUV420P10> &inputFrame,
             Frame<YUV400P16> &outputFrame) {
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
  std::transform(inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
                 outputFrame.getPlane(0).begin(),
                 [](Frame<YUV420P10>::base_type v) {
                   return (Frame<YUV400P16>::base_type)floor(
                       65536.f * (float(v) / 1024.f));
                 });
}

template <>
void convert(const Frame<YUV420P10> &inputFrame, Frame<YUV420P8> &outputFrame) {
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
  std::copy(inputFrame.getPlane(0).begin(), inputFrame.getPlane(0).end(),
            outputFrame.getPlane(0).begin());
}

template <>
void convert(const Frame<YUV420P16> &inputFrame, Frame<YUV420P8> &outputFrame) {
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
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
  assert(inputFrame.getHeight() == outputFrame.getHeight() &&
         inputFrame.getWidth() == outputFrame.getWidth());
  for (int i = 0; i < 3; i++) {
    std::transform(
        inputFrame.getPlane(i).begin(), inputFrame.getPlane(i).end(),
        outputFrame.getPlane(i).begin(), [](Frame<YUV420P16>::base_type v) {
          return (Frame<YUV420P10>::base_type)clamp(
              (float)floor(1024.f * (float(v) / 65536.f) + 0.5f), 0.f, 1023.f);
        });
  }
}

///////////////////////////////////////////////////////////
template <class FORMAT> std::string frameInfo(const Frame<FORMAT> &frame) {
  std::ostringstream oss;

  int N = frame.getNumberOfPlanes();
  oss << "planes=" << N << std::endl;

  for (int i = 0; i < N; ++i) {
    auto w = frame.getPlane(i).width();
    auto h = frame.getPlane(i).height();
    oss << "plane=" << i << ", w=" << w << ", h=" << h << std::endl;

    double firstValue = *frame.getPlane(i).row_begin(0);
    double n = 0.0, s = 0.0, ss = 0.0, mn = firstValue, mx = firstValue;
    for (auto y = 0u; y < h; ++y) {
      for (auto it = frame.getPlane(i).row_begin(y),
                itend = frame.getPlane(i).row_end(y);
           it != itend; ++it) {
        double v = static_cast<double>(*it);
        n++;
        s += v;
        ss += v * v;
        mn = std::min(mn, v);
        mx = std::max(mn, v);
      }
    }
    double mean = s / n;
    double var = ss / n - mean * mean;
    oss << "\tmin=" << mn << ", max=" << mx << ", mean=" << mean
        << ", sdev=" << sqrt(var) << std::endl;
  }
  oss << std::endl;

  return oss.str();
}

template <> std::string frameInfo(const TextureDepth16Frame &frame) {
  std::ostringstream oss;
  oss << "TextureDepth16Frame" << std::endl;
  oss << "Texture " << frameInfo(frame.first) << std::endl;
  oss << "Depth " << frameInfo(frame.second) << std::endl;

  return oss.str();
}

template <> std::string frameInfo(const MVD16Frame &frame) {
  std::ostringstream oss;
  for (auto i = 0u; i < frame.size(); ++i) {
    oss << "View " << i << std::endl;
    oss << frameInfo(frame[i]);
  }

  return oss.str();
}

template <> std::string frameInfo(const TextureDepth10Frame &frame) {
  std::ostringstream oss;
  oss << "TextureDepth10Frame" << std::endl;
  oss << "Texture " << frameInfo(frame.first) << std::endl;
  oss << "Depth " << frameInfo(frame.second) << std::endl;

  return oss.str();
}

template <> std::string frameInfo(const MVD10Frame &frame) {
  std::ostringstream oss;
  for (auto i = 0u; i < frame.size(); ++i) {
    oss << "View " << i << std::endl;
    oss << frameInfo(frame[i]);
  }

  return oss.str();
}

} // namespace TMIV::Common
