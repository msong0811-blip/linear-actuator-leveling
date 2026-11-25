#pragma once
#include <stdint.h>
namespace TravelLimits {
const int32_t MIN_POSITION=-6688, MAX_POSITION=4512, REFERENCE_POSITION=0;
const int32_t MAX_MOVE_PULSES=1600;
enum Result : uint8_t { OK, NEED_REFERENCE, BAD_POSITION, OUT_OF_RANGE, TOO_FAR };
inline Result validate(bool referenced,int32_t position,int32_t target) {
  if(!referenced) return NEED_REFERENCE;
  if(position<MIN_POSITION || position>MAX_POSITION) return BAD_POSITION;
  if(target<MIN_POSITION || target>MAX_POSITION) return OUT_OF_RANGE;
  int32_t distance=target-position; if(distance<0) distance=-distance;
  return distance>MAX_MOVE_PULSES ? TOO_FAR : OK;
}
inline bool nextStepAllowed(int32_t current,int32_t next) {
  return current>=MIN_POSITION && current<=MAX_POSITION && next>=MIN_POSITION && next<=MAX_POSITION && (next==current-1 || next==current+1);
}
}
