/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#ifndef TMIV_COMMON_APPLICATION_H
#define TMIV_COMMON_APPLICATION_H

#include "Factory.h"
#include "Json.h"

#include <ctime>

namespace TMIV::Common {
class Usage : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

class Application {
public:
  struct Option {
    std::string_view option;
    std::string_view description;
    bool multiple{};
    std::vector<std::string> values{};
  };
  using Options = std::vector<Option>;

  // Parse command-line arguments
  Application(char const *tool, std::vector<const char *> argv, Options options);

  Application(const Application &other) = delete;
  Application(Application &&other) = default;
  auto operator=(const Application &other) -> Application & = delete;
  auto operator=(Application &&other) -> Application & = default;
  virtual ~Application() = default;
  void startTime();
  void printTime() const;
  virtual void run() = 0;

protected:
  [[nodiscard]] auto json() const -> const Json &;

  [[nodiscard]] auto optionValues(std::string_view option) const
      -> const std::vector<std::string> &;

private:
  void add_file(const std::filesystem::path &path);
  void add_parameter(std::string key, std::string_view value);
  void add_stream(std::istream &stream);

  Json m_json;
  clock_t m_startTime{};
  Options m_options;
};
} // namespace TMIV::Common

#endif
