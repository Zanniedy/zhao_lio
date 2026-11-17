#pragma once

#include <stdexcept>
#include <string>

namespace config_utilities {

template <class Derived>
class Config {
 public:
  virtual ~Config() = default;
  const Derived& checkValid() const {
    checkParams();
    return static_cast<const Derived&>(*this);
  }

 protected:
  void setConfigName(const std::string& name) { name_ = name; }
  virtual void setupParamsAndPrinting() = 0;
  virtual void checkParams() const = 0;
  template <class Value>
  void setupParam(const std::string&, Value*, const std::string& = "") {}
  void checkParamCond(bool valid, const std::string& message) const {
    if (!valid) throw std::invalid_argument(name_ + ": " + message);
  }
  template <class Value, class Limit>
  void checkParamGE(Value value, Limit limit, const std::string& name) const {
    checkParamCond(value >= limit, name + " must meet lower bound");
  }
  template <class Value, class Limit>
  void checkParamGT(Value value, Limit limit, const std::string& name) const {
    checkParamCond(value > limit, name + " must exceed lower bound");
  }

 private:
  std::string name_;
};

}
