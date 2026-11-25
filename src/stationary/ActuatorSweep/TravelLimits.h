#pragma once
#include <stdint.h>

namespace TravelLimits {
const int32_t MIN_POSITION = -12000; // -75 mm at 160 pulses/mm.
const int32_t MAX_POSITION = -800;   // -5 mm.
const int32_t REFERENCE_POSITION = 0; // User-observed maximum extension.
const int32_t MAX_MOVE_PULSES = 1600; // Commissioning only: <=10 mm per command.

enum Result : uint8_t { OK, NEED_REFERENCE, BAD_POSITION, OUT_OF_RANGE, TOO_FAR };

inline Result validate(bool referenced, int32_t position, int32_t target) {
  if (!referenced) return NEED_REFERENCE;
  if (position < MIN_POSITION || position > REFERENCE_POSITION) return BAD_POSITION;
  if (target < MIN_POSITION || target > MAX_POSITION) return OUT_OF_RANGE;
  // Both operands are bounded above, so subtraction cannot overflow int32_t.
  int32_t distance = target - position;
  if (distance < 0) distance = -distance;
  if (distance > MAX_MOVE_PULSES) return TOO_FAR;
  return OK;
}

inline bool nextStepAllowed(int32_t current, int32_t next) {
  if (current < MIN_POSITION || current > REFERENCE_POSITION) return false;
  if (next < MIN_POSITION || next > REFERENCE_POSITION) return false;
  if (current > MAX_POSITION) return next == current - 1; // Inward entry only.
  return next <= MAX_POSITION && (next == current - 1 || next == current + 1);
}
}
