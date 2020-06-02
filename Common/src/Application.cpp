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

using namespace std;
using namespace std::string_literals;

namespace TMIV::Common {
const auto configFileOption = "-c"s;
const auto parameterOption = "-p"s;
const auto helpOption = "--help"s;

Application::Application(const char *tool, vector<const char *> argv) : m_startTime{} {
  auto take = [&argv]() {
    if (argv.empty()) {
      throw runtime_error("Missing a command-line argument");
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
      m_json.reset();
      break;
    } else {
      ostringstream what;
      what << "Stray argument \"" << option << "\"";
      throw runtime_error(what.str());
    }
  }

  if (!m_json) {
    ostringstream what;
    what << "Usage: " << tool << " [OPTIONS...] (-c CONFIGURATION|-p KEY VALUE)+\n";
    throw runtime_error(what.str());
  }
}

auto Application::json() const -> const Json & {
  assert(m_json);
  return *m_json;
}

void Application::add_file(const string &path) {
  ifstream stream(path);
  if (!stream.good()) {
    ostringstream what;
    what << "Failed to open configuration file \"" << path << "\" for reading\n";
    throw runtime_error(what.str());
  }
  add_stream(stream);
}

void Application::add_parameter(const string &key, string value) {
  stringstream stream;
  stream << "{ \"" << key << "\": ";
  if (value.empty()) {
    stream << "\"\"";
  } else if (value == "true" || value == "false" || value == "null" ||
             (isdigit(value.front()) != 0)) {
    stream << value;
  } else {
    stream << "\"" << value << "\"";
  }
  stream << " }";
  add_stream(stream);
}

void Application::add_stream(istream &stream) {
  auto root = Json{stream};
  if (m_json) {
    m_json->setOverrides(root);
  } else {
    m_json = make_shared<Json>(move(root));
  }
}

void Application::startTime() { m_startTime = clock(); }

void Application::printTime() const {
  auto executeTime = double(clock() - m_startTime) / CLOCKS_PER_SEC;
  cout << "Total Time: " << fixed << setprecision(3) << executeTime << " sec." << endl;
}

} // namespace TMIV::Common
