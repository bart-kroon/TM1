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

#ifndef _TMIV_COMMON_FACTORY_H_
#define _TMIV_COMMON_FACTORY_H_

#include <functional>
#include <map>
#include <string>
#include <iostream>

#include <TMIV/Common/Json.h>

namespace TMIV::Common {
// Factory that allows specifying data processing blocks in the configuration
// file. Because in general the parameters (of future implementations) are
// unknown the parameter is a Json.
template <class Interface> class Factory {
public:
  using Object = std::unique_ptr<Interface>;
  using Creator = std::function<Object(const Json &)>;

private:
  std::map<std::string, Creator> m_creators;

  Factory() = default;

public:
  Factory(const Factory &) = delete;
  Factory(Factory &&) = delete;
  Factory &operator=(const Factory &) = delete;
  Factory &operator=(Factory &&) = delete;

  // Return the singleton
  static Factory &getInstance() {
    static Factory instance;
    return instance;
  }

  // Create an object based on the method ID and JSON configuration
  Object create(const std::string &id, const Json &config) const {
    if (m_creators.count(id) == 0)  
    {
        std::cout << std::endl;
        for (auto entry : m_creators)
            std::cout << entry.first << std::endl;
        throw std::runtime_error("Error no registration for " + id);
    }
      
    return m_creators.at(id)(config);
  }

  // Register a new creator with a method ID
  template <class Derived> void registerAs(std::string id) {
    m_creators[id] = [](const Json &config) {
      return std::make_unique<Derived>(config);
    };
  }
};
} // namespace TMIV::Common

#endif