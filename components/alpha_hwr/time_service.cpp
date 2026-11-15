/**
 * Time Service Implementation
 * 
 * Manages pump real-time clock (RTC) for schedule execution and event logging.
 *
 * Frame shapes are bench-captured; see the header for both, and
 * send_set_clock_command() for how the write's fields were identified. The
 * Python reference this file was ported from described both of them wrongly,
 * so it is deliberately not cited as the authority here.
 */

#include "time_service.h"
#include "frame_builder.h"
#include "codec.h"
#include "esphome/core/log.h"
#include "alloc_compat.h"  // format_hex_pretty; see the header for the version guard
#include "esphome/core/time.h"
#include <cstring>

namespace esphome {
namespace alpha_hwr {
namespace services {

using namespace esphome::alpha_hwr::protocol;
using namespace esphome::alpha_hwr::core;

static const char* TAG = "alpha_hwr.time";

bool TimeService::resolve_wall_clock_(ESPTime &out) const {
#ifdef USE_TIME
  if (time_id_ == nullptr) return false;
  ESPTime now = time_id_->now();
  if (!clock_is_synced(now)) return false;
  out = now;
  return true;
#else
  // No time component means no timezone loaded either, so libc's answer here
  // would not merely be unvalidated -- it would be in the wrong zone. Refusing
  // is the honest answer, and after #270 it is the answer every caller gets.
  (void) out;
  return false;
#endif
}

bool TimeService::current_time(ESPTime &out) const {
  if (resolve_wall_clock_(out)) return true;
  // Same three causes as before, told apart the same way; only the resolution
  // moved. Kept distinct because "you did not configure a time source" and "the
  // one you configured has not answered yet" want different responses from
  // whoever reads the log, and clock_sync_gate.h acts on that distinction.
#ifdef USE_TIME
  if (time_id_ == nullptr) {
    ESP_LOGD(TAG, "time_id not configured - no wall clock to sync from");
  } else {
    ESP_LOGD(TAG, "System time not synced yet - nothing to write");
  }
#else
  ESP_LOGD(TAG, "time component not enabled in this build - no wall clock");
#endif
  return false;
}

uint32_t TimeService::now_unix() const {
  ESPTime now;
  if (!resolve_wall_clock_(now)) return 0;
  // clock_is_synced() floors the year at 2021, so a clock that passed it cannot
  // produce a negative or zero timestamp -- but the cast is from time_t, and
  // this returning 0 has to mean "no clock" and nothing else. Checking costs a
  // comparison and keeps the sentinel unambiguous rather than argued.
  if (now.timestamp <= 0) return 0;
  return static_cast<uint32_t>(now.timestamp);
}

bool TimeService::wall_clock_is_set() const {
  ESPTime now;
  return resolve_wall_clock_(now);
}



ESPTime TimeService::parse_clock_response(const uint8_t *data, size_t len) {
  // What the pump actually sends, captured on the bench (2026-08-15):
  //
  //   00 00 0C 07 EA 08 0F 14 26 37 48 00 06 00 01   -> 2026-08-15 20:38:55
  //   00 00 0C 07 EA 08 0F 14 27 08 13 00 06 00 01   -> 2026-08-15 20:39:08
  //
  // Fifteen bytes, from the object body onwards (the transport hands the
  // callback data+10, so these start at the size header):
  //   [Size(3BE)][Year(2BE)][Month][Day][Hour][Minute][Second][5 trailing]
  //
  // The first three bytes were called "[Status(2)][Length(1)]" here for a long
  // time and are neither: 00 00 0C is the ordinary three-byte size header, and
  // 0x0C = 12 is exactly the number of bytes that follow it. The old naming
  // read the same offsets, so the arithmetic below was always right and only
  // the labels were wrong.
  //
  // Of the five trailing bytes the first moved between the two samples (0x48,
  // 0x13) and the last four did not. Unidentified; none of them are read.
  //
  // The >= 10 floor is kept rather than tightened to the observed 15: it is the
  // minimum this parser needs (3 header + 7 datetime), and a firmware that
  // sends a shorter tail should still yield a usable clock.
  if (len < 10) {
    ESP_LOGW(TAG, "Clock response too short: %zu bytes (expected >= 10)", len);
    return ESPTime();
  }
  
  ESP_LOGD(TAG, "Raw clock data: %s (%zu bytes)", format_hex_pretty(data, len).c_str(), len);
  
  // Low 16 bits of the 3-byte size header; logged only, as a sanity read.
  uint16_t declared_size = decode_uint16_be(data, 1);
  
  // Skip the 3-byte size header; the datetime starts at byte 3.
  const uint8_t *payload = data + 3;
  size_t payload_len = len - 3;
  
  if (payload_len < 7) {
    ESP_LOGW(TAG, "Clock payload too short: %zu bytes (expected >= 7)", payload_len);
    return ESPTime();
  }
  
  // Year is big-endian uint16
  uint16_t year = decode_uint16_be(payload, 0);
  uint8_t month = payload[2];
  uint8_t day = payload[3];
  uint8_t hour = payload[4];
  uint8_t minute = payload[5];
  uint8_t second = payload[6];
  
  ESP_LOGD(TAG, "Parsed clock: %04d-%02d-%02d %02d:%02d:%02d, declared_size=%u",
           year, month, day, hour, minute, second, declared_size);
  
  // Handle unset/invalid clock
  if (year < 1970 || year > 2100 || month == 0 || month > 12 || day == 0 || day > 31) {
    ESP_LOGW(TAG, "Pump clock is unset or invalid: %04d-%02d-%02d", year, month, day);
    return ESPTime();  // Return invalid time
  }
  
  // Create ESPTime. Value-initialized: the fields set below are every field
  // ESPTime has except is_dst, and leaving that one indeterminate is a read of
  // an uninitialized bool waiting for the first caller who looks at it.
  ESPTime pump_time{};
  pump_time.year = year;
  pump_time.month = month;
  pump_time.day_of_month = day;
  pump_time.hour = hour;
  pump_time.minute = minute;
  pump_time.second = second;
  pump_time.day_of_week = 1;  // Not provided by pump, set to Monday
  pump_time.day_of_year = 1;  // Not provided by pump
  
  // Calculate Unix timestamp treating pump time as local time in our configured timezone
  // Start with epoch local to initialize valid baseline (avoids is_valid() check failing)
  ESPTime base = ESPTime::from_epoch_local(0);
  base.year = year;
  base.month = month;
  base.day_of_month = day;
  base.hour = hour;
  base.minute = minute;
  base.second = second;
  
  // Use ESPHome's timezone engine to parse these local fields into a UTC timestamp
  // (Works in static context because ESPTime timezone configuration is global in ESPHome)
  base.recalc_timestamp_local();
  pump_time.timestamp = base.timestamp;
  
  // time_t width varies by platform (long long on ESP-IDF, long on many hosts):
  // widen to long long so one format string is correct everywhere.
  ESP_LOGD(TAG, "ESPTime created: timestamp=%lld, is_valid=%d",
           static_cast<long long>(pump_time.timestamp), pump_time.is_valid());
  
  return pump_time;
}

}  // namespace services
}  // namespace alpha_hwr
}  // namespace esphome
