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

#include <TMIV/Common/Json.h>

#include <cctype>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace TMIV::Common {
namespace {
void skipWhitespaceAndLineComments(std::istream &stream) {
  for (;;) {
    // Skip whitespace
    while (!stream.eof() && (isspace(stream.peek()) != 0)) {
      stream.get();
    }

    if (stream.eof() || stream.peek() != '/') {
      // No line comment
      return;
    }

    // Skip line comment
    stream.get();
    if (stream.eof()) {
      throw std::runtime_error("Stray '/' at end of file");
    }
    if (stream.peek() != '/') {
      std::ostringstream what;
      what << "Stray character 0x" << std::hex << stream.peek() << " at std::end of file\n";
      throw std::runtime_error(what.str());
    }
    while (!stream.eof() && stream.peek() != '\n') {
      stream.get();
    }
  }
}

void matchCharacter(std::istream &stream, std::istream::int_type expected) {
  auto actual = stream.get();

  if (actual != expected) {
    std::ostringstream what;
    what << "Expected '" << static_cast<char>(expected) << "' but found '"
         << static_cast<char>(actual) << "' (0x" << std::hex << actual << ")";
    throw std::runtime_error(what.str());
  }
}

void matchText(std::istream &stream, std::string const &text) {
  for (auto ch : text) {
    matchCharacter(stream, ch);
  }
}
} // namespace

static auto readValue(std::istream &stream) -> std::shared_ptr<impl::Value>;

namespace impl {
struct Value {
  explicit Value(Json::Type type) : type(type) {}
  Value(Value const &) = default;
  Value(Value &&) = default;
  auto operator=(Value const &) -> Value & = default;
  auto operator=(Value &&) -> Value & = default;
  virtual ~Value() = default;

  Json::Type type;
};

struct String : public Value {
public:
  explicit String(std::istream &stream) : Value(Json::Type::string) {
    matchCharacter(stream, '"');
    auto ch = stream.get();

    while (ch != '"') {
      if (ch == '\\') {
        switch (stream.get()) {
        case '"':
          value.push_back('"');
          break;
        case '\\':
          value.push_back('\\');
          break;
        case '/':
          value.push_back('/');
          break;
        case 'b':
          value.push_back('\b');
          break;
        case 'f':
          value.push_back('\f');
          break;
        case 'n':
          value.push_back('\n');
          break;
        case 'r':
          value.push_back('\r');
          break;
        case 't':
          value.push_back('\t');
          break;
        case 'u':
          throw std::runtime_error("JSON parser: unicode std::string escaping not yet implemented");
        default:
          throw std::runtime_error("JSON parser: invalid std::string escape character");
        }
      } else {
        value.push_back(static_cast<char>(ch));
      }
      ch = stream.get();
    }
  }

  std::string value;
};

struct Number : public Value {
  explicit Number(std::istream &stream) : Value(Json::Type::number) { stream >> value; }

  double value{};
};

struct Object : public Value {
  explicit Object(std::istream &stream) : Value(Json::Type::object) {
    matchCharacter(stream, '{');
    skipWhitespaceAndLineComments(stream);

    while (stream.peek() != '}') {
      if (!value.empty()) {
        matchCharacter(stream, ',');
        skipWhitespaceAndLineComments(stream);
      }

      auto key = String(stream);
      skipWhitespaceAndLineComments(stream);
      matchCharacter(stream, ':');
      skipWhitespaceAndLineComments(stream);
      value[key.value] = readValue(stream);
      skipWhitespaceAndLineComments(stream);
    }

    stream.get();
  }

  std::map<std::string, std::shared_ptr<Value>> value;
};

struct Array : public Value {
  explicit Array(std::istream &stream) : Value(Json::Type::array) {
    matchCharacter(stream, '[');
    skipWhitespaceAndLineComments(stream);

    if (stream.peek() != ']') {
      value.push_back(readValue(stream));
      skipWhitespaceAndLineComments(stream);

      while (stream.peek() == ',') {
        stream.get();
        value.push_back(readValue(stream));
        skipWhitespaceAndLineComments(stream);
      }
    }

    matchCharacter(stream, ']');
  }

  std::vector<std::shared_ptr<Value>> value;
};

struct Bool : public Value {
public:
  explicit Bool(std::istream &stream) : Value(Json::Type::boolean) {
    value = stream.peek() == 't';
    matchText(stream, value ? "true" : "false");
  }

  bool value;
};

struct Null : public Value {
  Null() : Value(Json::Type::null) {}

  explicit Null(std::istream &stream) : Null() { matchText(stream, "null"); }
};
} // namespace impl

Json::Json() : m_value(new impl::Null) {}

Json::Json(std::shared_ptr<impl::Value> value) : m_value(std::move(value)) {}

Json::Json(std::istream &stream) {
  try {
    stream.exceptions(std::ios::badbit | std::ios::failbit);
    auto value = readValue(stream);
    skipWhitespaceAndLineComments(stream);

    if (!stream.eof()) {
      auto ch = stream.get();
      std::ostringstream what;
      what << "Stray character " << static_cast<char>(ch) << " (0x" << std::ios::hex << ch << ")";
      throw std::runtime_error(what.str());
    }

    m_value = std::move(value);
  } catch (std::runtime_error &e) {
    throw std::runtime_error(std::string("JSON parser: ") + e.what());
  }
}

void Json::setOverrides(const Json &overrides) {
  if (type() == Type::object && overrides.type() == Type::object) {
    for (const auto &kvp : dynamic_cast<const impl::Object &>(*overrides.m_value).value) {
      dynamic_cast<impl::Object &>(*m_value).value[kvp.first] = kvp.second;
    }
  } else {
    throw std::runtime_error("Overrides should be a JSON object, e.g. {...}");
  }
}

auto Json::type() const -> Json::Type { return m_value->type; }

auto Json::optional(std::string const &key) const -> Json {
  try {
    return Json{dynamic_cast<impl::Object &>(*m_value).value.at(key)};
  } catch (std::out_of_range &) {
    return {};
  } catch (std::bad_cast &) {
    std::ostringstream what;
    what << "JSON parser: Querying optional key '" << key << "', but node is not an object";
    throw std::runtime_error(what.str());
  }
}

auto Json::require(std::string const &key) const -> Json {
  auto node = optional(key);
  if (node.type() != Type::null) {
    return node;
  }
  std::ostringstream stream;
  stream << "JSON parser: Parameter " << key << " is required but missing";
  throw std::runtime_error(stream.str());
}

auto Json::isPresent(std::string const &key) const -> bool {
  auto node = optional(key);
  return node.type() != Type::null;
}

auto Json::at(size_t index) const -> Json {
  if (type() != Type::array) {
    throw std::runtime_error("JSON parser: Expected an array");
  }
  return Json{dynamic_cast<impl::Array &>(*m_value).value.at(index)};
}

auto Json::size() const -> size_t {
  switch (type()) {
  case Type::array:
    return dynamic_cast<impl::Array &>(*m_value).value.size();
  case Type::object:
    return dynamic_cast<impl::Object &>(*m_value).value.size();
  default:
    throw std::runtime_error("JSON parser: Expected an array or object");
  }
}

auto Json::asDouble() const -> double {
  if (type() != Type::number) {
    throw std::runtime_error("JSON parser: Expected a number");
  }
  return dynamic_cast<impl::Number &>(*m_value).value;
}

auto Json::asFloat() const -> float { return static_cast<float>(asDouble()); }

auto Json::asInt() const -> int {
  auto value = asDouble();
  auto rounded = static_cast<int>(std::lround(value));
  auto error = value - rounded;
  constexpr auto eps = 1e-6;
  if (error > eps) {
    throw std::runtime_error("JSON parser: Expected an integer value");
  }
  return rounded;
}

auto Json::asString() const -> std::string const & {
  if (type() != Type::string) {
    throw std::runtime_error("JSON parser: Expected a std::string");
  }
  return dynamic_cast<impl::String &>(*m_value).value;
}

auto Json::asBool() const -> bool {
  if (type() != Type::boolean) {
    throw std::runtime_error("JSON parser: Expected a boolean");
  }
  return dynamic_cast<impl::Bool &>(*m_value).value;
}

auto Json::asStringVector() const -> std::vector<std::string> {
  auto v = std::vector<std::string>();
  v.reserve(size());
  for (size_t i = 0; i < size(); ++i) {
    v.emplace_back(at(i).asString());
  }
  return v;
}

Json::operator bool() const {
  switch (type()) {
  case Type::null:
    return false;
  case Type::boolean:
    return asBool();
  default:
    return true;
  }
}

static auto readValue(std::istream &stream) -> std::shared_ptr<impl::Value> {
  skipWhitespaceAndLineComments(stream);
  auto ch = stream.peek();

  switch (ch) {
  case '{':
    return std::make_shared<impl::Object>(stream);
  case '[':
    return std::make_shared<impl::Array>(stream);
  case '"':
    return std::make_shared<impl::String>(stream);
  case 't':
  case 'f':
    return std::make_shared<impl::Bool>(stream);
  case 'n':
    return std::make_shared<impl::Null>(stream);
  default:
    break;
  }

  if (ch == '-' || (isdigit(ch) != 0)) {
    return std::make_shared<impl::Number>(stream);
  }

  std::ostringstream what;
  what << "Invalid character " << static_cast<char>(ch) << " (0x" << std::ios::hex << ch << ")";
  throw std::runtime_error(what.str());
}
} // namespace TMIV::Common
