#pragma once

#include <cstdint>

namespace uplink_boundary_policy
{
  constexpr uint32_t elapsedUs(uint32_t now_us, uint32_t started_at_us)
  {
    // unsigned差分によりmicros()の一回のwrapを安全に扱う。
    return now_us - started_at_us;
  }

  constexpr bool isFresh(uint32_t now_us, uint32_t received_at_us,
                         uint32_t maximum_age_us)
  {
    return elapsedUs(now_us, received_at_us) <= maximum_age_us;
  }

  constexpr bool deadlineExpiredMs(uint32_t now_ms, uint32_t started_at_ms,
                                   uint32_t timeout_ms)
  {
    // unsigned差分によりmillis()の一回のwrapを安全に扱う。
    return now_ms - started_at_ms >= timeout_ms;
  }

  constexpr uint8_t resetPeriodicStreak()
  {
    return 0;
  }

  constexpr uint8_t advancePeriodicStreak(uint8_t current)
  {
    return current == UINT8_MAX ? current
                                : static_cast<uint8_t>(current + 1U);
  }

  constexpr bool periodicModeActive(uint8_t consecutive_periodic,
                                    uint8_t activation_count)
  {
    return activation_count != 0 &&
           consecutive_periodic >= activation_count;
  }
} // namespace uplink_boundary_policy
