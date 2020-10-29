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

#include <TMIV/Common/Application.h>

#include <cassert>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace std::string_literals;

namespace TMIV::Common {
const auto configFileOption = "-c"s;
const auto parameterOption = "-p"s;
const auto helpOption = "--help"s;

Application::Application(const char *tool, std::vector<const char *> argv) : m_startTime{} {
  auto take = [&argv]() {
    if (argv.empty()) {
      throw std::runtime_error("Missing a command-line argument");
    }
    const auto *result = argv.front();
    argv.erase(argv.begin());
    return result;
  };

  take();

  while (!argv.empty()) {
    const auto *option = take();
    if (configFileOption == option) {
      add_file(take());
    } else if (parameterOption == option) {
      const auto *arg1 = take();
      const auto *arg2 = take();
      add_parameter(arg1, arg2);
    } else if (helpOption == option) {
      m_json = Json{};
      break;
    } else {
      std::ostringstream what;
      what << "Stray argument \"" << option << "\"";
      throw std::runtime_error(what.str());
    }
  }

  if (!m_json) {
    std::ostringstream what;
    what << "Usage: " << tool << " [OPTIONS...] (-c CONFIGURATION|-p KEY VALUE)+\n";
    throw std::runtime_error(what.str());
  }
}

auto Application::json() const -> const Json & {
  assert(m_json);
  return m_json;
}

void Application::add_file(const std::string &path) {
  std::ifstream stream(path);
  if (!stream.good()) {
    std::ostringstream what;
    what << "Failed to open configuration file \"" << path << "\" for reading\n";
    throw std::runtime_error(what.str());
  }
  add_stream(stream);
}

void Application::add_parameter(std::string key, std::string_view value) {
  auto json = Json{value};
  try {
    json = Json::parse(value);
  } catch (std::runtime_error &e) {
    fmt::print("WARNING: Interpreting command-line {} parameter overide as a string \"{}\" because "
               "the JSON parser didn't like it:\n  {}\n",
               key, value, e.what());
  }

  m_json.update(Json{std::in_place_type<Json::Object>, std::pair{std::move(key), json}});
}

void Application::add_stream(std::istream &stream) {
  if (m_json) {
    m_json.update(Json::loadFrom(stream));
  } else {
    m_json = Json::loadFrom(stream);
  }
}
void Application::startTime() { m_startTime = clock(); }

void Application::printTime() const {
  auto executeTime =
      (static_cast<double>(clock()) - static_cast<double>(m_startTime)) / CLOCKS_PER_SEC;
  std::cout << "Total Time: " << std::fixed << std::setprecision(3) << executeTime << " sec."
            << std::endl;
}
} // namespace TMIV::Common
