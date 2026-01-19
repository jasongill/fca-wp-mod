// ///////////////////////////////////////////////////////////// //
// Hardware abstraction layer for all different supported boards //
// ///////////////////////////////////////////////////////////// //
#include "board_declarations.h"
#include "boards/common.h"

// ///// Board definition and detection ///// //
#include "drivers/harness.h"
#include "drivers/fan.h"
#include "drivers/rtc.h"
#include "drivers/clock_source.h"
#include "boards/white.h"

void detect_board_type(void) {
  hw_type = HW_TYPE_WHITE_PANDA;
  current_board = &board_white;
}


// ///// Configuration detection ///// //
bool has_external_debug_serial = 0;

void detect_configuration(void) {
  // detect if external serial debugging is present
  has_external_debug_serial = detect_with_pull(GPIOA, 3, PULL_DOWN);
}

// ///// Board functions ///// //
bool board_has_gps(void) {
  return false;
}

bool board_has_gmlan(void) {
  return true;
}

bool board_has_obd(void) {
  return false;
}

bool board_has_lin(void) {
  return true;
}

bool board_has_rtc(void) {
  return false;
}

bool board_has_relay(void) {
  return false;
}
