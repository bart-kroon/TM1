/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

#ifndef _TMIV_COMMON_APPLICATION_H_
#define _TMIV_COMMON_APPLICATION_H_

#include "Factory.h"
#include "Json.h"

namespace TMIV::Common {
class Application {
public:
  // Parse command-line arguments
  Application(char const *tool, std::vector<const char *> /*argv*/);

  Application(const Application &other) = delete;
  Application(Application &&other) = default;
  Application &operator=(const Application &other) = delete;
  Application &operator=(Application &&other) = default;
  virtual ~Application() = default;

  virtual void run() = 0;

protected:
  const Json &json() const;

  // Use the configuration file with a factory to create a component/module
  template <class Interface, typename... Args>
  auto create(Args &&... next) const {
    auto result =
        getComponentParentAndName(json(), std::forward<Args>(next)...);
    return Factory<Interface>::getInstance().create(std::move(result.second),
                                                    json(), result.first);
  }

private:
  void add_file(const std::string &path);
  void add_parameter(const std::string &key, std::string value);
  void add_stream(std::istream &stream);
  std::pair<Json, std::string>
  getComponentParentAndName(const Json &node, const std::string &name) const {
    return {node, name};
  }

  template <typename... Args>
  std::pair<Json, std::string>
  getComponentParentAndName(const Json &node, const std::string &first,
                            Args &&... next) const {
    return getComponentParentAndName(
        node.require(node.require(first + "Method").asString()),
        std::forward<Args>(next)...);
  }

  std::shared_ptr<Json> m_json;
};
} // namespace TMIV::Common

#endif
