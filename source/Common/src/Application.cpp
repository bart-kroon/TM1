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

#include <TMIV/Common/Application.h>

#include <TMIV/Common/Formatters.h>
#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/Common/Thread.h>
#include <TMIV/Common/Version.h>
#include <TMIV/Common/verify.h>

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

using namespace std::string_view_literals;

namespace TMIV::Common {
Application::Application(const char *tool, std::vector<const char *> argv, Options options)
    : m_options{std::move(options)} {
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
    std::string_view option = take();
    if ("-c"sv == option) {
      add_file(take());
    } else if ("-p"sv == option) {
      const auto *arg1 = take();
      const auto *arg2 = take();
      add_parameter(arg1, arg2);
    } else if ("-j"sv == option) {
      const auto arg = std::atoi(take());
      if (0 < arg) {
        Common::threadCount() = arg;
      } else {
        throw std::runtime_error("The -j option has as argument a positive number");
      }
    } else if ("-V"sv == option) {
      [&take]() {
        const auto *const arg = take();

        for (const auto level : allLogLevels) {
          if (fmt::format("{}", level) == arg) {
            return changeMaxLogLevel(level);
          }
        }
        throw std::runtime_error(fmt::format("Unknown log level {}", arg));
      }();
    } else if ("--version"sv == option) {
      Common::logInfo("TMIV {} version {}", tool, version);
      exit(0);
    } else if ("--help"sv == option) {
      m_json = Json{};
      break;
    } else { // search for application-specific options
      auto o = std::find_if(m_options.begin(), m_options.end(),
                            [option](const auto &o) { return o.option == option; });
      if (o == m_options.end()) {
        throw std::runtime_error(
            fmt::format("Stray argument or unknown option \"{}\" (try --help)", option));
      }
      if (!o->multiple && !o->values.empty()) {
        throw std::runtime_error(fmt::format("Option {} may not occur multiple times", o->option));
      }
      o->values.emplace_back(take());
    }
  }

  if (!m_json || std::any_of(m_options.cbegin(), m_options.cend(),
                             [](const auto &o) { return !o.multiple && o.values.empty(); })) {
    std::ostringstream what;
    what << "Usage: Tmiv" << tool
         << " [OPTIONS...] (-c CONFIGURATION)+ (-p KEY VALUE)* [-j THREAD_COUNT] [-V";
    auto sep = ' ';

    for (const auto level : allLogLevels) {
      what << sep << level;
      sep = '|';
    }
    what << "]\n\nOptions are:";
    for (const auto &o : m_options) {
      what << fmt::format("\n  {:3} {:47} {}", o.option, o.description,
                          o.multiple ? "zero or more allowed" : "required exactly once");
    }
    what << R"(

NOTE 1: When the same parameter is provided multiple times on the command-line,
        through -c or -p, then the right-most argument has precedence.
NOTE 2: The default thread count is equal to the logical processor count of the
        system. Use -j 1 to disable parallel processing.)";
    throw Usage{what.str()};
  }
}

auto Application::json() const -> const Json & {
  POSTCONDITION(m_json);
  return m_json;
}

auto Application::optionValues(std::string_view option) const -> const std::vector<std::string> & {
  auto o = std::find_if(m_options.cbegin(), m_options.cend(),
                        [=](const auto &o) { return o.option == option; });
  PRECONDITION(o != m_options.cend());
  return o->values;
}

void Application::add_file(const std::filesystem::path &path) {
  std::ifstream stream(path);
  if (!stream.good()) {
    throw std::runtime_error(fmt::format("Failed to open {} for reading (with current path {})",
                                         path, std::filesystem::current_path()));
  }
  add_stream(stream);
}

void Application::add_parameter(std::string key, std::string_view value) {
  auto json = Json{value};
  try {
    json = Json::parse(value);
  } catch (std::runtime_error & /* unused */) {
  }
  Common::logInfo("Override {}: {}", key, json.format());

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
  logInfo("Note that the total time reported directly below is CPU time, and the logging timestamp "
          "is wall time. Typically the CPU time is reported.");
  logInfo("Total Time: {:.3f} sec.", executeTime);
}
} // namespace TMIV::Common
