// ============================================================================
// Safety System - CAN message validation and control hooks
// ============================================================================

const int MAX_WRONG_COUNTERS = 5;
const uint8_t MAX_MISSED_MSGS = 10U;

// sample struct that keeps 3 samples in memory
struct sample_t {
  int values[6];
  int min;
  int max;
} sample_t_default = {{0}, 0, 0};

// safety code requires floats
struct lookup_t {
  float x[3];
  float y[3];
};

typedef struct {
  int addr;
  int bus;
  int len;
} CanMsg;

typedef struct {
  const int addr;
  const int bus;
  const int len;
  const bool check_checksum;
  const uint8_t max_counter;
  const uint32_t expected_timestep;
} CanMsgCheck;

typedef struct {
  const CanMsgCheck msg[3];
  bool msg_seen;
  int index;
  bool valid_checksum;
  int wrong_counters;
  uint8_t last_counter;
  uint32_t last_timestamp;
  bool lagging;
} AddrCheckStruct;

// Function prototypes
int safety_rx_hook(CAN_FIFOMailBox_TypeDef *to_push);
int safety_tx_hook(CAN_FIFOMailBox_TypeDef *to_send);
int safety_tx_lin_hook(int lin_num, uint8_t *data, int len);
uint32_t get_ts_elapsed(uint32_t ts, uint32_t ts_last);
int to_signed(int d, int bits);
void update_sample(struct sample_t *sample, int sample_new);
bool max_limit_check(int val, const int MAX, const int MIN);
bool dist_to_meas_check(int val, int val_last, struct sample_t *val_meas,
  const int MAX_RATE_UP, const int MAX_RATE_DOWN, const int MAX_ERROR);
bool driver_limit_check(int val, int val_last, struct sample_t *val_driver,
  const int MAX, const int MAX_RATE_UP, const int MAX_RATE_DOWN,
  const int MAX_ALLOWANCE, const int DRIVER_FACTOR);
bool rt_rate_limit_check(int val, int val_last, const int MAX_RT_DELTA);
float interpolate(struct lookup_t xy, float x);
void gen_crc_lookup_table(uint8_t poly, uint8_t crc_lut[]);
bool msg_allowed(CAN_FIFOMailBox_TypeDef *to_send, const CanMsg msg_list[], int len);
int get_addr_check_index(CAN_FIFOMailBox_TypeDef *to_push, AddrCheckStruct addr_list[], const int len);
void update_counter(AddrCheckStruct addr_list[], int index, uint8_t counter);
void update_addr_timestamp(AddrCheckStruct addr_list[], int index);
bool is_msg_valid(AddrCheckStruct addr_list[], int index);
bool addr_safety_check(CAN_FIFOMailBox_TypeDef *to_push,
                       AddrCheckStruct *addr_check,
                       const int addr_check_len,
                       uint8_t (*get_checksum)(CAN_FIFOMailBox_TypeDef *to_push),
                       uint8_t (*compute_checksum)(CAN_FIFOMailBox_TypeDef *to_push),
                       uint8_t (*get_counter)(CAN_FIFOMailBox_TypeDef *to_push));
void generic_rx_checks(bool stock_ecu_detected);
void relay_malfunction_set(void);
void relay_malfunction_reset(void);

typedef void (*safety_hook_init)(int16_t param);
typedef int (*rx_hook)(CAN_FIFOMailBox_TypeDef *to_push);
typedef int (*tx_hook)(CAN_FIFOMailBox_TypeDef *to_send);
typedef int (*tx_lin_hook)(int lin_num, uint8_t *data, int len);
typedef int (*fwd_hook)(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd);

typedef struct {
  safety_hook_init init;
  rx_hook rx;
  tx_hook tx;
  tx_lin_hook tx_lin;
  fwd_hook fwd;
  AddrCheckStruct *addr_check;
  const int addr_check_len;
} safety_hooks;

void safety_tick(const safety_hooks *hooks);

// ============================================================================
// Global State Variables
// ============================================================================

bool controls_allowed = false;
bool relay_malfunction = false;
bool gas_interceptor_detected = false;
int gas_interceptor_prev = 0;
bool gas_pressed = false;
bool gas_pressed_prev = false;
bool brake_pressed = false;
bool brake_pressed_prev = false;
bool cruise_engaged_prev = false;
float vehicle_speed = 0;
bool vehicle_moving = false;

// for safety modes with torque steering control
int desired_torque_last = 0;
int rt_torque_last = 0;
struct sample_t torque_meas;
struct sample_t torque_driver;
uint32_t ts_last = 0;

// for safety modes with angle steering control
uint32_t ts_angle_last = 0;
int desired_angle_last = 0;
struct sample_t angle_meas;

// Unsafe mode flags
#define UNSAFE_DISABLE_DISENGAGE_ON_GAS 1
#define UNSAFE_DISABLE_STOCK_AEB 2
#define UNSAFE_RAISE_LONGITUDINAL_LIMITS_TO_ISO_MAX 8
int unsafe_mode = 0;

uint32_t safety_mode_cnt = 0U;
const uint32_t RELAY_TRNS_TIMEOUT = 1U;

// ============================================================================
// Include FCA Mod (EPS speed override, APA spoofing, ACC control)
// ============================================================================
#include "../fca_mod.h"

// ============================================================================
// Safety Mode: No Output
// ============================================================================

static void nooutput_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();
}

static int nooutput_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  UNUSED(to_send);
  return false;
}

static int nooutput_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  UNUSED(lin_num);
  UNUSED(data);
  UNUSED(len);
  return false;
}

static int default_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  UNUSED(to_fwd);
  UNUSED(bus_num);
  return -1;
}

const safety_hooks nooutput_hooks = {
  .init = nooutput_init,
  .rx = fca_rx_hook,
  .tx = nooutput_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = default_fwd_hook,
};

// ============================================================================
// Safety Mode: All Output (Debug)
// ============================================================================

static void alloutput_init(int16_t param) {
  UNUSED(param);
  controls_allowed = true;
  relay_malfunction_reset();
}

static int alloutput_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  UNUSED(to_send);
  return true;
}

static int alloutput_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  UNUSED(lin_num);
  UNUSED(data);
  UNUSED(len);
  return true;
}

const safety_hooks alloutput_hooks = {
  .init = alloutput_init,
  .rx = fca_rx_hook,
  .tx = alloutput_tx_hook,
  .tx_lin = alloutput_tx_lin_hook,
  .fwd = default_fwd_hook,
};

// ============================================================================
// Safety Mode: Chrysler/FCA
// ============================================================================

const int CHRYSLER_MAX_STEER = 261;
const int CHRYSLER_MAX_RT_DELTA = 112;
const uint32_t CHRYSLER_RT_INTERVAL = 250000;
const int CHRYSLER_MAX_RATE_UP = 3;
const int CHRYSLER_MAX_RATE_DOWN = 3;
const int CHRYSLER_MAX_TORQUE_ERROR = 80;
const int CHRYSLER_GAS_THRSLD = 30;
const int CHRYSLER_STANDSTILL_THRSLD = 10;
const CanMsg CHRYSLER_TX_MSGS[] = {{571, 0, 3}, {658, 0, 6}, {678, 0, 8}};

AddrCheckStruct chrysler_rx_checks[] = {
  {.msg = {{544, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 10000U}}},
  {.msg = {{514, 0, 8, .check_checksum = false, .max_counter = 0U, .expected_timestep = 10000U}}},
  {.msg = {{500, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}}},
  {.msg = {{308, 0, 8, .check_checksum = false, .max_counter = 15U, .expected_timestep = 20000U}}},
  {.msg = {{320, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}}},
};
const int CHRYSLER_RX_CHECK_LEN = sizeof(chrysler_rx_checks) / sizeof(chrysler_rx_checks[0]);

static uint8_t chrysler_get_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  int checksum_byte = GET_LEN(to_push) - 1;
  return (uint8_t)(GET_BYTE(to_push, checksum_byte));
}

static uint8_t chrysler_compute_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
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

static uint8_t chrysler_get_counter(CAN_FIFOMailBox_TypeDef *to_push) {
  return (uint8_t)(GET_BYTE(to_push, 6) >> 4);
}

static int chrysler_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  bool valid = addr_safety_check(to_push, chrysler_rx_checks, CHRYSLER_RX_CHECK_LEN,
                                 chrysler_get_checksum, chrysler_compute_checksum,
                                 chrysler_get_counter);

  if (valid && (GET_BUS(to_push) == 0)) {
    int addr = GET_ADDR(to_push);

    // Measured eps torque
    if (addr == 544) {
      int torque_meas_new = ((GET_BYTE(to_push, 4) & 0x7U) << 8) + GET_BYTE(to_push, 5) - 1024U;
      update_sample(&torque_meas, torque_meas_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if (addr == 500) {
      int cruise_engaged = ((GET_BYTE(to_push, 2) & 0x38) >> 3) == 7;
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    // update speed
    if (addr == 514) {
      int speed_l = (GET_BYTE(to_push, 0) << 4) + (GET_BYTE(to_push, 1) >> 4);
      int speed_r = (GET_BYTE(to_push, 2) << 4) + (GET_BYTE(to_push, 3) >> 4);
      vehicle_speed = (speed_l + speed_r) / 2;
      vehicle_moving = (int)vehicle_speed > CHRYSLER_STANDSTILL_THRSLD;
    }

    // exit controls on rising edge of gas press
    if (addr == 308) {
      gas_pressed = ((GET_BYTE(to_push, 5) & 0x7F) != 0) && ((int)vehicle_speed > CHRYSLER_GAS_THRSLD);
    }

    // exit controls on rising edge of brake press
    if (addr == 320) {
      brake_pressed = (GET_BYTE(to_push, 0) & 0x7) == 5;
      if (brake_pressed && (!brake_pressed_prev || vehicle_moving)) {
        controls_allowed = 0;
      }
      brake_pressed_prev = brake_pressed;
    }

    generic_rx_checks((addr == 0x292));
  }
  return valid;
}

static int chrysler_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  int tx = 1;
  int addr = GET_ADDR(to_send);

  if (!msg_allowed(to_send, CHRYSLER_TX_MSGS, sizeof(CHRYSLER_TX_MSGS) / sizeof(CHRYSLER_TX_MSGS[0]))) {
    tx = 0;
  }

  if (relay_malfunction) {
    tx = 0;
  }

  // LKA STEER
  if (addr == 0x292) {
    int desired_torque = ((GET_BYTE(to_send, 0) & 0x7U) << 8) + GET_BYTE(to_send, 1) - 1024U;
    uint32_t ts = TIM2->CNT;
    bool violation = 0;

    if (controls_allowed) {
      violation |= max_limit_check(desired_torque, CHRYSLER_MAX_STEER, -CHRYSLER_MAX_STEER);
      violation |= dist_to_meas_check(desired_torque, desired_torque_last,
        &torque_meas, CHRYSLER_MAX_RATE_UP, CHRYSLER_MAX_RATE_DOWN, CHRYSLER_MAX_TORQUE_ERROR);
      desired_torque_last = desired_torque;
      violation |= rt_rate_limit_check(desired_torque, rt_torque_last, CHRYSLER_MAX_RT_DELTA);

      uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
      if (ts_elapsed > CHRYSLER_RT_INTERVAL) {
        rt_torque_last = desired_torque;
        ts_last = ts;
      }
    }

    if (!controls_allowed && (desired_torque != 0)) {
      violation = 1;
    }

    if (violation || !controls_allowed) {
      desired_torque_last = 0;
      rt_torque_last = 0;
      ts_last = ts;
    }

    if (violation) {
      tx = 0;
    }
  }

  // FORCE CANCEL: only the cancel button press is allowed
  if (addr == 571) {
    if ((GET_BYTE(to_send, 0) != 1) || ((GET_BYTE(to_send, 1) & 1) == 1)) {
      tx = 0;
    }
  }

  return tx;
}

static int chrysler_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  if (!relay_malfunction) {
    // forward CAN 0 -> 2 so stock LKAS camera sees messages
    if (bus_num == 0) {
      bus_fwd = 2;
    }
    // forward all messages from camera except LKAS_COMMAND and LKAS_HUD
    if ((bus_num == 2) && (addr != 658) && (addr != 678)) {
      bus_fwd = 0;
    }
  }
  return bus_fwd;
}

const safety_hooks chrysler_hooks = {
  .init = nooutput_init,
  .rx = chrysler_rx_hook,
  .tx = chrysler_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = chrysler_fwd_hook,
  .addr_check = chrysler_rx_checks,
  .addr_check_len = sizeof(chrysler_rx_checks) / sizeof(chrysler_rx_checks[0]),
};

// ============================================================================
// Safety Mode: ELM327 (OBD Diagnostics)
// ============================================================================

static int elm327_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  int tx = 1;
  int addr = GET_ADDR(to_send);
  int len = GET_LEN(to_send);

  if (len != 8) {
    tx = 0;
  }

  if ((addr != 0x18DB33F1) && ((addr & 0x1FFF00FF) != 0x18DA00F1) &&
      ((addr & 0x1FFFFF00) != 0x700)) {
    tx = 0;
  }
  return tx;
}

static int elm327_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  int tx = 1;
  if (lin_num != 0) {
    tx = 0;
  }
  if ((len < 5) || (len > 11)) {
    tx = 0;
  }
  if (!(((data[0] & 0xF8U) == 0xC0U) && ((data[0] & 0x07U) != 0U) &&
        (data[1] == 0x33U) && (data[2] == 0xF1U))) {
    tx = 0;
  }
  return tx;
}

const safety_hooks elm327_hooks = {
  .init = nooutput_init,
  .rx = fca_rx_hook,
  .tx = elm327_tx_hook,
  .tx_lin = elm327_tx_lin_hook,
  .fwd = default_fwd_hook,
};

// ============================================================================
// Safety Mode Registry
// ============================================================================

#define SAFETY_SILENT 0U
#define SAFETY_ELM327 3U
#define SAFETY_CHRYSLER 9U
#define SAFETY_ALLOUTPUT 17U
#define SAFETY_NOOUTPUT 19U

uint16_t current_safety_mode = SAFETY_SILENT;
const safety_hooks *current_hooks = &nooutput_hooks;

int safety_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  return current_hooks->rx(to_push);
}

int safety_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  return current_hooks->tx(to_send);
}

int safety_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  return current_hooks->tx_lin(lin_num, data, len);
}

int safety_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  return current_hooks->fwd(bus_num, to_fwd);
}

typedef struct {
  uint16_t id;
  const safety_hooks *hooks;
} safety_hook_config;

const safety_hook_config safety_hook_registry[] = {
  {SAFETY_SILENT, &nooutput_hooks},
  {SAFETY_ELM327, &elm327_hooks},
  {SAFETY_CHRYSLER, &chrysler_hooks},
  {SAFETY_NOOUTPUT, &nooutput_hooks},
#ifdef ALLOW_DEBUG
  {SAFETY_ALLOUTPUT, &alloutput_hooks},
#endif
};

int set_safety_hooks(uint16_t mode, int16_t param) {
  safety_mode_cnt = 0U;
  relay_malfunction = false;
  gas_interceptor_detected = false;
  gas_interceptor_prev = 0;
  gas_pressed = false;
  gas_pressed_prev = false;
  brake_pressed = false;
  brake_pressed_prev = false;
  cruise_engaged_prev = false;
  vehicle_speed = 0;
  vehicle_moving = false;
  desired_torque_last = 0;
  rt_torque_last = 0;
  ts_angle_last = 0;
  desired_angle_last = 0;
  ts_last = 0;

  torque_meas.max = 0;
  torque_meas.max = 0;
  torque_driver.min = 0;
  torque_driver.max = 0;
  angle_meas.min = 0;
  angle_meas.max = 0;

  int set_status = -1;
  int hook_config_count = sizeof(safety_hook_registry) / sizeof(safety_hook_config);
  for (int i = 0; i < hook_config_count; i++) {
    if (safety_hook_registry[i].id == mode) {
      current_hooks = safety_hook_registry[i].hooks;
      current_safety_mode = safety_hook_registry[i].id;
      set_status = 0;
    }

    for (int j = 0; j < safety_hook_registry[i].hooks->addr_check_len; j++) {
      safety_hook_registry[i].hooks->addr_check[j].index = 0;
      safety_hook_registry[i].hooks->addr_check[j].msg_seen = false;
    }
  }
  if ((set_status == 0) && (current_hooks->init != NULL)) {
    current_hooks->init(param);
  }
  return set_status;
}

// ============================================================================
// Utility Functions
// ============================================================================

void gen_crc_lookup_table(uint8_t poly, uint8_t crc_lut[]) {
  for (int i = 0; i < 256; i++) {
    uint8_t crc = i;
    for (int j = 0; j < 8; j++) {
      if ((crc & 0x80U) != 0U)
        crc = (uint8_t)((crc << 1) ^ poly);
      else
        crc <<= 1;
    }
    crc_lut[i] = crc;
  }
}

bool msg_allowed(CAN_FIFOMailBox_TypeDef *to_send, const CanMsg msg_list[], int len) {
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);
  int length = GET_LEN(to_send);

  bool allowed = false;
  for (int i = 0; i < len; i++) {
    if ((addr == msg_list[i].addr) && (bus == msg_list[i].bus) && (length == msg_list[i].len)) {
      allowed = true;
      break;
    }
  }
  return allowed;
}

uint32_t get_ts_elapsed(uint32_t ts, uint32_t ts_last) {
  return ts - ts_last;
}

int get_addr_check_index(CAN_FIFOMailBox_TypeDef *to_push, AddrCheckStruct addr_list[], const int len) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);
  int length = GET_LEN(to_push);

  int index = -1;
  for (int i = 0; i < len; i++) {
    if (!addr_list[i].msg_seen) {
      for (uint8_t j = 0U; addr_list[i].msg[j].addr != 0; j++) {
        if ((addr == addr_list[i].msg[j].addr) && (bus == addr_list[i].msg[j].bus) &&
              (length == addr_list[i].msg[j].len)) {
          addr_list[i].index = j;
          addr_list[i].msg_seen = true;
          break;
        }
      }
    }

    int idx = addr_list[i].index;
    if ((addr == addr_list[i].msg[idx].addr) && (bus == addr_list[i].msg[idx].bus) &&
        (length == addr_list[i].msg[idx].len)) {
      index = i;
      break;
    }
  }
  return index;
}

void safety_tick(const safety_hooks *hooks) {
  uint32_t ts = TIM2->CNT;
  if (hooks->addr_check != NULL) {
    for (int i = 0; i < hooks->addr_check_len; i++) {
      uint32_t elapsed_time = get_ts_elapsed(ts, hooks->addr_check[i].last_timestamp);
      bool lagging = elapsed_time > MAX(hooks->addr_check[i].msg[hooks->addr_check[i].index].expected_timestep * MAX_MISSED_MSGS, 1e6);
      hooks->addr_check[i].lagging = lagging;
      if (lagging) {
        controls_allowed = 0;
      }
    }
  }
}

void update_counter(AddrCheckStruct addr_list[], int index, uint8_t counter) {
  if (index != -1) {
    uint8_t expected_counter = (addr_list[index].last_counter + 1U) % (addr_list[index].msg[addr_list[index].index].max_counter + 1U);
    addr_list[index].wrong_counters += (expected_counter == counter) ? -1 : 1;
    addr_list[index].wrong_counters = MAX(MIN(addr_list[index].wrong_counters, MAX_WRONG_COUNTERS), 0);
    addr_list[index].last_counter = counter;
  }
}

bool is_msg_valid(AddrCheckStruct addr_list[], int index) {
  bool valid = true;
  if (index != -1) {
    if ((!addr_list[index].valid_checksum) || (addr_list[index].wrong_counters >= MAX_WRONG_COUNTERS)) {
      valid = false;
      controls_allowed = 0;
    }
  }
  return valid;
}

void update_addr_timestamp(AddrCheckStruct addr_list[], int index) {
  if (index != -1) {
    uint32_t ts = TIM2->CNT;
    addr_list[index].last_timestamp = ts;
  }
}

bool addr_safety_check(CAN_FIFOMailBox_TypeDef *to_push,
                       AddrCheckStruct *rx_checks,
                       const int rx_checks_len,
                       uint8_t (*get_checksum)(CAN_FIFOMailBox_TypeDef *to_push),
                       uint8_t (*compute_checksum)(CAN_FIFOMailBox_TypeDef *to_push),
                       uint8_t (*get_counter)(CAN_FIFOMailBox_TypeDef *to_push)) {

  int index = get_addr_check_index(to_push, rx_checks, rx_checks_len);
  update_addr_timestamp(rx_checks, index);

  if (index != -1) {
    if ((get_checksum != NULL) && (compute_checksum != NULL) && rx_checks[index].msg[rx_checks[index].index].check_checksum) {
      uint8_t checksum = get_checksum(to_push);
      uint8_t checksum_comp = compute_checksum(to_push);
      rx_checks[index].valid_checksum = checksum_comp == checksum;
    } else {
      rx_checks[index].valid_checksum = true;
    }

    if ((get_counter != NULL) && (rx_checks[index].msg[rx_checks[index].index].max_counter > 0U)) {
      uint8_t counter = get_counter(to_push);
      update_counter(rx_checks, index, counter);
    } else {
      rx_checks[index].wrong_counters = 0U;
    }
  }
  return is_msg_valid(rx_checks, index);
}

void generic_rx_checks(bool stock_ecu_detected) {
  if (gas_pressed && !gas_pressed_prev && !(unsafe_mode & UNSAFE_DISABLE_DISENGAGE_ON_GAS)) {
    controls_allowed = 0;
  }
  gas_pressed_prev = gas_pressed;

  if (brake_pressed && (!brake_pressed_prev || vehicle_moving)) {
    controls_allowed = 0;
  }
  brake_pressed_prev = brake_pressed;

  if ((safety_mode_cnt > RELAY_TRNS_TIMEOUT) && stock_ecu_detected) {
    relay_malfunction_set();
  }
}

void relay_malfunction_set(void) {
  relay_malfunction = true;
  fault_occurred(FAULT_RELAY_MALFUNCTION);
}

void relay_malfunction_reset(void) {
  relay_malfunction = false;
  fault_recovered(FAULT_RELAY_MALFUNCTION);
}

int to_signed(int d, int bits) {
  int d_signed = d;
  if (d >= (1 << MAX((bits - 1), 0))) {
    d_signed = d - (1 << MAX(bits, 0));
  }
  return d_signed;
}

void update_sample(struct sample_t *sample, int sample_new) {
  int sample_size = sizeof(sample->values) / sizeof(sample->values[0]);
  for (int i = sample_size - 1; i > 0; i--) {
    sample->values[i] = sample->values[i-1];
  }
  sample->values[0] = sample_new;

  sample->min = sample->values[0];
  sample->max = sample->values[0];
  for (int i = 1; i < sample_size; i++) {
    if (sample->values[i] < sample->min) {
      sample->min = sample->values[i];
    }
    if (sample->values[i] > sample->max) {
      sample->max = sample->values[i];
    }
  }
}

bool max_limit_check(int val, const int MAX_VAL, const int MIN_VAL) {
  return (val > MAX_VAL) || (val < MIN_VAL);
}

bool dist_to_meas_check(int val, int val_last, struct sample_t *val_meas,
  const int MAX_RATE_UP, const int MAX_RATE_DOWN, const int MAX_ERROR) {

  int highest_allowed_rl = MAX(val_last, 0) + MAX_RATE_UP;
  int lowest_allowed_rl = MIN(val_last, 0) - MAX_RATE_UP;

  int highest_allowed = MIN(highest_allowed_rl, MAX(val_last - MAX_RATE_DOWN, MAX(val_meas->max, 0) + MAX_ERROR));
  int lowest_allowed = MAX(lowest_allowed_rl, MIN(val_last + MAX_RATE_DOWN, MIN(val_meas->min, 0) - MAX_ERROR));

  return (val < lowest_allowed) || (val > highest_allowed);
}

bool driver_limit_check(int val, int val_last, struct sample_t *val_driver,
  const int MAX_VAL, const int MAX_RATE_UP, const int MAX_RATE_DOWN,
  const int MAX_ALLOWANCE, const int DRIVER_FACTOR) {

  int highest_allowed_rl = MAX(val_last, 0) + MAX_RATE_UP;
  int lowest_allowed_rl = MIN(val_last, 0) - MAX_RATE_UP;

  int driver_max_limit = MAX_VAL + (MAX_ALLOWANCE + val_driver->max) * DRIVER_FACTOR;
  int driver_min_limit = -MAX_VAL + (-MAX_ALLOWANCE + val_driver->min) * DRIVER_FACTOR;

  int highest_allowed = MIN(highest_allowed_rl, MAX(val_last - MAX_RATE_DOWN, MAX(driver_max_limit, 0)));
  int lowest_allowed = MAX(lowest_allowed_rl, MIN(val_last + MAX_RATE_DOWN, MIN(driver_min_limit, 0)));

  return (val < lowest_allowed) || (val > highest_allowed);
}

bool rt_rate_limit_check(int val, int val_last, const int MAX_RT_DELTA) {
  int highest_val = MAX(val_last, 0) + MAX_RT_DELTA;
  int lowest_val = MIN(val_last, 0) - MAX_RT_DELTA;
  return (val < lowest_val) || (val > highest_val);
}

float interpolate(struct lookup_t xy, float x) {
  int size = sizeof(xy.x) / sizeof(xy.x[0]);
  float ret = xy.y[size - 1];

  if (x <= xy.x[0]) {
    ret = xy.y[0];
  } else {
    for (int i = 0; i < (size - 1); i++) {
      if (x < xy.x[i+1]) {
        float x0 = xy.x[i];
        float y0 = xy.y[i];
        float dx = xy.x[i+1] - x0;
        float dy = xy.y[i+1] - y0;
        if (dx <= 0.) {
          dx = 0.0001;
        }
        ret = (dy * (x - x0) / dx) + y0;
        break;
      }
    }
  }
  return ret;
}
