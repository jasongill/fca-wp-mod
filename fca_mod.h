// ============================================================================
// FCA White Panda Mod - EPS Speed Override for Low-Speed Steering
// ============================================================================
//
// This mod enables LKAS/APA steering control at any vehicle speed on FCA
// (Chrysler/Dodge/Jeep/Ram) vehicles by spoofing the speed signal sent to
// the EPS (Electric Power Steering) module.
//
// How it works:
// - The stock EPS has speed cutoffs that disable LKAS below ~65 kph
// - This mod intercepts the speed message (CAN ID 0x11C / 284) and replaces
//   the speed value with a fake speed that keeps the EPS in LKAS/APA mode
// - When steer_type == 1 (LKAS): Reports 65 kph to EPS (LKAS enable threshold)
// - When steer_type == 2 (APA):  Reports 0 kph to EPS (full low-speed control)
// - When steer_type == 3 (OFF):  Passes through real speed (stock behavior)
//
// The mod also supports:
// - APA (Automatic Parking Assist) mode spoofing for low-speed steering
// - ACC (Adaptive Cruise Control) message passthrough for openpilot longitudinal
//
// ============================================================================

// ============================================================================
// FCA Mod State Variables
// ============================================================================

// Steering control state
bool is_op_active = false;          // Is openpilot actively steering?
int lkas_torq = 1024;               // LKAS torque command (1024 = center/zero)
int steer_type = 3;                 // 1=LKAS, 2=APA, 3=OFF (passthrough)
int steer_control_type = 0;         // Control type from openpilot

// Longitudinal (ACC) control state
bool is_oplong_enabled = false;     // Is openpilot longitudinal enabled?
int acc_set_speed_kph = 255;
int acc_set_speed_mph = 255;
int cruise_state = 0;
int cruise_icon = 0;
int lead_dist = 255;
int acc_text_msg = 0;
bool acc_text_req = false;

// ACC command state
bool acc_stop = false;
bool acc_go = false;
int acc_decel_cmd = 4094;
bool acc_available = false;
bool acc_enabled = false;
bool acc_brk_prep = false;
int command_type = 0;

// ACC engine torque request
bool acc_eng_req = 0;
int acc_torq = 7767;

// Stock ACC state (for collision detection passthrough)
bool org_acc_available = false;
int org_cmd_type = 0;
bool org_brk_pul = false;
bool org_collision_active = false;

// Message counters for timeout detection
int counter_284_502 = 0;
int counter_284 = 0;
int counter_284_658 = 0;
int counter_658 = 0;
int counter_502 = 0;

// ============================================================================
// FCA CAN Checksum
// ============================================================================

// FCA CAN checksum algorithm
// Reference: http://illmatics.com/Remote%20Car%20Hacking.pdf
static uint8_t fca_compute_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  uint8_t checksum = 0xFF;
  int len = GET_LEN(to_push);
  for (int j = 0; j < (len - 1); j++) {
    uint8_t shift = 0x80;
    uint8_t curr = (uint8_t)GET_BYTE(to_push, j);
    for (int i = 0; i < 8; i++) {
      uint8_t bit_sum = curr & shift;
      uint8_t temp_chk = checksum & 0x80U;
      if (bit_sum != 0U) {
        bit_sum = 0x1C;
        if (temp_chk != 0U) {
          bit_sum = 1;
        }
        checksum = checksum << 1;
        temp_chk = checksum | 1U;
        bit_sum ^= temp_chk;
      } else {
        if (temp_chk != 0U) {
          bit_sum = 0x1D;
        }
        checksum = checksum << 1;
        bit_sum ^= checksum;
      }
      checksum = bit_sum;
      shift = shift >> 1;
    }
  }
  return ~checksum;
}

// ============================================================================
// EPS Speed Override - THE CORE MOD
// ============================================================================

// Overrides the speed signal sent to the EPS module.
// This is what enables steering at any speed.
//
// CAN ID 0x11C (284) - SPEED_1 message
// Bytes 4-5: Vehicle speed (little-endian, 128x factor)
// Byte 7: Checksum
//
// Speed thresholds:
// - LKAS enable speed: 65 kph (EPS allows LKAS above this)
// - APA enable speed:  0 kph  (EPS allows full steering in APA mode)
static void fca_eps_speed_override(CAN_FIFOMailBox_TypeDef *to_fwd) {
  int kph_factor = 128;
  int lkas_enable_speed = 65 * kph_factor;  // 65 kph threshold
  int apa_enable_speed = 0 * kph_factor;    // 0 kph for APA mode
  int veh_speed = GET_BYTE(to_fwd, 4) | GET_BYTE(to_fwd, 5) << 8;
  int eps_cutoff_speed = veh_speed;  // Default: pass through real speed

  if (steer_type == 2) {
    // APA mode: report 0 kph to enable full low-speed steering
    eps_cutoff_speed = apa_enable_speed >> 8 | ((apa_enable_speed << 8) & 0xFFFF);
  } else if (steer_type == 1) {
    // LKAS mode: report 65 kph to keep LKAS enabled
    eps_cutoff_speed = lkas_enable_speed >> 8 | ((lkas_enable_speed << 8) & 0xFFFF);
  }
  // steer_type == 3: pass through real speed (stock behavior)

  to_fwd->RDHR &= 0x00FF0000;  // Clear speed and checksum, keep counter
  to_fwd->RDHR |= eps_cutoff_speed;
  int crc = fca_compute_checksum(to_fwd);
  to_fwd->RDHR |= (((crc << 8) << 8) << 8);
}

// ============================================================================
// APA Mode Spoofing Functions
// ============================================================================

// Spoof transmission gear to Reverse (APA requires reverse gear)
static void fca_spoof_trans_reverse(CAN_FIFOMailBox_TypeDef *to_fwd) {
  if (steer_type == 2) {
    int gear_R = 0xB;
    to_fwd->RDLR &= 0xFFFFF0FF;
    to_fwd->RDLR |= gear_R << 8;
  }
}

// Spoof shifter position to Reverse
static void fca_spoof_shifter_reverse(CAN_FIFOMailBox_TypeDef *to_fwd) {
  if (steer_type == 2) {
    int shifter_R = 0x1;
    to_fwd->RDLR &= 0xFFFFFFE0;
    to_fwd->RDLR |= shifter_R << 2;
  }
}

// Clear reverse indicator
static void fca_clear_reverse_indicator(CAN_FIFOMailBox_TypeDef *to_fwd) {
  if (steer_type == 2) {
    to_fwd->RDLR &= 0xFFFFFFEF;
  }
}

// Zero wheel speed for APA mode
static void fca_zero_wheel_speed(CAN_FIFOMailBox_TypeDef *to_fwd) {
  if (steer_type == 2) {
    to_fwd->RDLR &= 0x00000000;
  }
}

// Zero counter message for APA mode
static void fca_zero_counter_msg(CAN_FIFOMailBox_TypeDef *to_fwd) {
  if (steer_type == 2) {
    to_fwd->RDLR &= 0x00000000;
    to_fwd->RDHR &= 0x00000000;
  }
}

// Send APA steering torque command
// Converts LKAS torque to APA torque format
static void fca_send_apa_torque(CAN_FIFOMailBox_TypeDef *to_fwd) {
  int multi = 4;  // Torque multiplier
  int apa_torq = ((lkas_torq - 1024) * multi / 4) + 1024;

  if ((steer_type == 2) && is_op_active) {
    to_fwd->RDLR &= 0x00000000;
    to_fwd->RDLR |= 0x50;                      // APA request = true
    to_fwd->RDLR |= 0x20 << 8 << 8;            // APA type = 1
    to_fwd->RDLR |= apa_torq >> 8;             // Torque high byte
    to_fwd->RDLR |= (apa_torq & 0xFF) << 8;    // Torque low byte
  }
  to_fwd->RDHR &= 0x00FF0000;  // Keep counter
  int crc = fca_compute_checksum(to_fwd);
  to_fwd->RDHR |= (((crc << 8) << 8) << 8);
}

// ============================================================================
// ACC (Adaptive Cruise Control) Message Modification
// ============================================================================

// Modify ACC deceleration command
static void fca_acc_decel_msg(CAN_FIFOMailBox_TypeDef *to_fwd) {
  if (is_oplong_enabled && !org_collision_active) {
    to_fwd->RDLR &= 0x00000000;
    to_fwd->RDHR &= 0x00FD0080;  // Keep counter

    to_fwd->RDLR |= acc_stop << 5;
    to_fwd->RDLR |= acc_go << 6;
    to_fwd->RDLR |= ((acc_decel_cmd >> 8) << 8) << 8;
    to_fwd->RDLR |= ((acc_available << 8) << 8) << 4;
    to_fwd->RDLR |= ((acc_enabled << 8) << 8) << 5;
    to_fwd->RDLR |= ((acc_decel_cmd << 8) << 8) << 8;

    to_fwd->RDHR |= command_type << 4;
    to_fwd->RDHR |= ((acc_brk_prep << 8) << 8) << 1;

    int crc = fca_compute_checksum(to_fwd);
    to_fwd->RDHR |= (((crc << 8) << 8) << 8);
  }
}

// Modify ACC dashboard display message
static void fca_acc_dash_msg(CAN_FIFOMailBox_TypeDef *to_fwd) {
  if (is_oplong_enabled && !org_collision_active) {
    to_fwd->RDLR &= 0x7C000000;
    to_fwd->RDHR &= 0x00C8C000;
    to_fwd->RDLR |= acc_text_msg;
    to_fwd->RDLR |= acc_set_speed_kph << 8;
    to_fwd->RDLR |= (acc_set_speed_mph << 8) << 8;
    to_fwd->RDLR |= (((acc_text_req << 8) << 8) << 8) << 7;

    to_fwd->RDHR |= cruise_state << 4;
    to_fwd->RDHR |= cruise_icon << 8;
    to_fwd->RDHR |= ((lead_dist << 8) << 8) << 8;
  }
}

// Modify ACC acceleration/engine torque request
static void fca_acc_accel_msg(CAN_FIFOMailBox_TypeDef *to_fwd) {
  if (is_oplong_enabled && !org_collision_active) {
    to_fwd->RDHR &= 0x00FF0000;  // Keep counter
    to_fwd->RDHR |= (acc_eng_req << 7);
    to_fwd->RDHR |= (acc_torq >> 8) | ((acc_torq << 8) & 0xFFFF);
    int crc = fca_compute_checksum(to_fwd);
    to_fwd->RDHR |= (((crc << 8) << 8) << 8);
  }
}

// Modify wheel button message (to disable stock ACC)
static void fca_wheel_button_msg(CAN_FIFOMailBox_TypeDef *to_fwd) {
  if (is_oplong_enabled) {
    to_fwd->RDLR &= 0x00F000;  // Keep counter
    if (org_acc_available) {
      to_fwd->RDLR |= 0x80;  // Send ACC button to disable stock ACC
    }
    int crc = fca_compute_checksum(to_fwd);
    to_fwd->RDLR |= ((crc << 8) << 8);
  }
}

// ============================================================================
// Openpilot Command Reception
// ============================================================================

// Receives commands from openpilot via CAN bus 0
// Parses steering torque, control mode, and ACC commands
int fca_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  int addr = GET_ADDR(to_push);
  int bus_num = GET_BUS(to_push);

  // CAN ID 658 (0x292): LKAS_COMMAND from openpilot
  // Contains steering torque and control type
  if ((addr == 658) && (bus_num == 0)) {
    is_op_active = (GET_BYTE(to_push, 0) >> 4) & 0x1;
    lkas_torq = ((GET_BYTE(to_push, 0) & 0x7) << 8) | GET_BYTE(to_push, 1);
    counter_658 += 1;

    steer_control_type = (GET_BYTE(to_push, 0) >> 7) & 0x1;
    if (steer_control_type == 1) {
      // Steering control type determines LKAS vs APA mode
      if ((GET_BYTE(to_push, 0) >> 6) & 0x1) {
        steer_type = 1;  // LKAS mode
      } else {
        steer_type = 3;  // OFF (passthrough)
      }
    }
  }

  // CAN ID 284 (0x11C): SPEED_1 - used for timeout detection
  if ((addr == 284) && (bus_num == 0)) {
    // Check for openpilot longitudinal timeout
    if (counter_502 > 0) {
      counter_284_502 += 1;
      if (counter_284_502 - counter_502 > 25) {
        is_oplong_enabled = false;
        acc_enabled = false;
        counter_502 = 0;
        counter_284_502 = 0;
      }
    }

    // Check for openpilot steering timeout
    if (counter_658 > 0) {
      counter_284_658 += 2;
      if (counter_284_658 - counter_658 > 25) {
        is_op_active = false;
        steer_type = 3;  // Revert to passthrough on timeout
        steer_control_type = 0;
        counter_658 = 0;
        counter_284_658 = 0;
      }
    }
  }

  // CAN ID 502 (0x1F6): ACC decel command from openpilot
  if ((addr == 502) && (bus_num == 0)) {
    acc_stop = (GET_BYTE(to_push, 0) >> 5) & 0x1;
    acc_go = (GET_BYTE(to_push, 0) >> 6) & 0x1;
    acc_available = (GET_BYTE(to_push, 2) >> 4) & 0x1;
    acc_enabled = (GET_BYTE(to_push, 2) >> 5) & 0x1;
    acc_decel_cmd = ((GET_BYTE(to_push, 2) & 0xF) << 8) | GET_BYTE(to_push, 3);
    command_type = (GET_BYTE(to_push, 4) >> 4) & 0x7;
    acc_brk_prep = (GET_BYTE(to_push, 6) >> 1) & 0x1;
    counter_502 += 1;
  }

  // CAN ID 503 (0x1F7): ACC dashboard info from openpilot
  if ((addr == 503) && (bus_num == 0)) {
    is_oplong_enabled = GET_BYTE(to_push, 3) & 0x1;
    acc_text_msg = GET_BYTE(to_push, 0);
    acc_set_speed_kph = GET_BYTE(to_push, 1);
    acc_set_speed_mph = GET_BYTE(to_push, 2);
    cruise_state = (GET_BYTE(to_push, 4) >> 4) & 0x7;
    cruise_icon = GET_BYTE(to_push, 5) & 0x3F;
    lead_dist = GET_BYTE(to_push, 7);
    acc_text_req = GET_BYTE(to_push, 3) >> 7;
  }

  // CAN ID 626 (0x272): ACC engine torque request from openpilot
  if ((addr == 626) && (bus_num == 0)) {
    acc_eng_req = (GET_BYTE(to_push, 4) >> 7) & 0x1;
    acc_torq = (GET_BYTE(to_push, 4) & 0x7F) << 8 | GET_BYTE(to_push, 5);
  }

  // CAN ID 500 (0x1F4): ACC status - determine steer mode from ACC state
  if ((addr == 500) && (bus_num == 0) && (steer_control_type == 0)) {
    if (GET_BYTE(to_push, 2) >> 4 & 0x1) {
      steer_type = 1;  // LKAS mode when ACC ready
    } else {
      steer_type = 3;  // OFF
    }
  }

  // CAN ID 500 from bus 1: Stock ACC state (for collision detection passthrough)
  if ((addr == 500) && (bus_num == 1)) {
    if (is_oplong_enabled) {
      org_acc_available = (GET_BYTE(to_push, 2) >> 4) & 0x1;
      org_cmd_type = (GET_BYTE(to_push, 4) >> 4) & 0x7;
      org_brk_pul = GET_BYTE(to_push, 6) & 0x1;
      // Pass through stock collision avoidance
      org_collision_active = org_brk_pul || (org_cmd_type > 1);
    } else {
      org_acc_available = false;
    }
  }

  return true;
}

// Send heartbeat message
void fca_heartbeat(void) {
  CAN1->sTxMailBox[0].TDLR = 0x00;
  CAN1->sTxMailBox[0].TDTR = 4;
  CAN1->sTxMailBox[0].TIR = (0x4FFU << 21) | 1U;
}
