#pragma once
#include <cstdint>
#include <string>

namespace fwis {

enum class Mode : uint8_t {
  Ackermann = 0,  // m_A: front/rear Ackermann steering
  Lateral   = 1,  // m_L: pure lateral motion (phi = pi/2)
  Diagonal  = 2,  // m_P: parallel/diagonal motion (phi in [-pi/2, pi/2])
  Spin      = 3,  // m_S: zero-radius pivot, theta-only motion (no reverse)
};

inline std::string mode_name(Mode m) {
  switch (m) {
    case Mode::Ackermann: return "A";
    case Mode::Lateral:   return "L";
    case Mode::Diagonal:  return "P";
    case Mode::Spin:      return "S";
  }
  return "?";
}

}  // namespace fwis
