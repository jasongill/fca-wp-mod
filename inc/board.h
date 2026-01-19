// ///////////////////////////////////////////////////////////// //
// Hardware abstraction layer for White Panda                    //
// ///////////////////////////////////////////////////////////// //

// ******************** Prototypes ********************
typedef void (*board_init)(void);
typedef void (*board_enable_can_transceiver)(uint8_t transceiver, bool enabled);
typedef void (*board_enable_can_transceivers)(bool enabled);
typedef void (*board_set_led)(uint8_t color, bool enabled);
typedef void (*board_set_usb_power_mode)(uint8_t mode);
typedef void (*board_set_gps_mode)(uint8_t mode);
typedef void (*board_set_can_mode)(uint8_t mode);
typedef void (*board_usb_power_mode_tick)(uint32_t uptime);
typedef bool (*board_check_ignition)(void);
typedef uint32_t (*board_read_current)(void);
typedef void (*board_set_ir_power)(uint8_t percentage);
typedef void (*board_set_fan_power)(uint8_t percentage);
typedef void (*board_set_phone_power)(bool enabled);
typedef void (*board_set_clock_source_mode)(uint8_t mode);
typedef void (*board_set_siren)(bool enabled);

struct harness_configuration {
  const bool has_harness;
  GPIO_TypeDef *GPIO_SBU1;
  GPIO_TypeDef *GPIO_SBU2;
  GPIO_TypeDef *GPIO_relay_SBU1;
  GPIO_TypeDef *GPIO_relay_SBU2;
  uint8_t pin_SBU1;
  uint8_t pin_SBU2;
  uint8_t pin_relay_SBU1;
  uint8_t pin_relay_SBU2;
  uint8_t adc_channel_SBU1;
  uint8_t adc_channel_SBU2;
};

struct board {
  const char *board_type;
  const struct harness_configuration *harness_config;
  board_init init;
  board_enable_can_transceiver enable_can_transceiver;
  board_enable_can_transceivers enable_can_transceivers;
  board_set_led set_led;
  board_set_usb_power_mode set_usb_power_mode;
  board_set_gps_mode set_gps_mode;
  board_set_can_mode set_can_mode;
  board_usb_power_mode_tick usb_power_mode_tick;
  board_check_ignition check_ignition;
  board_read_current read_current;
  board_set_ir_power set_ir_power;
  board_set_fan_power set_fan_power;
  board_set_phone_power set_phone_power;
  board_set_clock_source_mode set_clock_source_mode;
  board_set_siren set_siren;
};

// ******************* Definitions ********************
#define HW_TYPE_UNKNOWN 0U
#define HW_TYPE_WHITE_PANDA 1U

// Harness status (White Panda has no harness)
#define HARNESS_STATUS_NC 0U
#define HARNESS_STATUS_NORMAL 1U
#define HARNESS_STATUS_FLIPPED 2U
uint8_t car_harness_status = HARNESS_STATUS_NC;

// LED colors
#define LED_RED 0U
#define LED_GREEN 1U
#define LED_BLUE 2U

// USB power modes
#define USB_POWER_NONE 0U
#define USB_POWER_CLIENT 1U
#define USB_POWER_CDP 2U
#define USB_POWER_DCP 3U

// GPS modes
#define GPS_DISABLED 0U
#define GPS_ENABLED 1U
#define GPS_BOOTMODE 2U

// CAN modes
#define CAN_MODE_NORMAL 0U
#define CAN_MODE_GMLAN_CAN2 1U
#define CAN_MODE_GMLAN_CAN3 2U
#define CAN_MODE_OBD_CAN2 3U

// ********************* Globals **********************
uint8_t usb_power_mode = USB_POWER_NONE;

// ************ Board function prototypes *************
bool board_has_gps(void);
bool board_has_gmlan(void);
bool board_has_obd(void);
bool board_has_lin(void);
bool board_has_rtc(void);
bool board_has_relay(void);

// ///////////////// Common GPIO ///////////////////// //
#include "stm32f4xx_hal_gpio_ex.h"

void common_init_gpio(void){
  // pull low to hold ESP in reset
  // enable OTG out tied to ground
  GPIOA->ODR = 0;
  GPIOB->ODR = 0;
  GPIOA->PUPDR = 0;
  GPIOB->AFR[0] = 0;
  GPIOB->AFR[1] = 0;

  // C2: Voltage sense line
  set_gpio_mode(GPIOC, 2, MODE_ANALOG);

  // A11,A12: USB
  set_gpio_alternate(GPIOA, 11, GPIO_AF10_OTG_FS);
  set_gpio_alternate(GPIOA, 12, GPIO_AF10_OTG_FS);
  GPIOA->OSPEEDR = GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12;

  // A9,A10: USART 1 for talking to the GPS
  set_gpio_alternate(GPIOA, 9, GPIO_AF7_USART1);
  set_gpio_alternate(GPIOA, 10, GPIO_AF7_USART1);

   // B8,B9: CAN 1
  set_gpio_alternate(GPIOB, 8, GPIO_AF8_CAN1);
  set_gpio_alternate(GPIOB, 9, GPIO_AF8_CAN1);
}

void peripherals_init(void){
  // enable GPIOB, UART2, CAN, USB clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  #ifdef PANDA
    RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
  #endif
  RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
  RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;
  #ifdef CAN3
    RCC->APB1ENR |= RCC_APB1ENR_CAN3EN;
  #endif
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // main counter
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // pedal and fan PWM
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;  // gmlan_alt and IR PWM
  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;  // k-line init
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;  // interrupt timer
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;   // for RTC config
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  // clock source timer
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;  // slow loop
}

#define PULL_EFFECTIVE_DELAY 4096
bool detect_with_pull(GPIO_TypeDef *GPIO, int pin, int mode) {
  set_gpio_mode(GPIO, pin, MODE_INPUT);
  set_gpio_pullup(GPIO, pin, mode);
  for (volatile int i=0; i<PULL_EFFECTIVE_DELAY; i++);
  bool ret = get_gpio_input(GPIO, pin);
  set_gpio_pullup(GPIO, pin, PULL_NONE);
  return ret;
}

// ///////////////////// Drivers ///////////////////// //
#include "fan.h"
#include "rtc.h"
#include "clock_source.h"

// Harness relay control (White Panda has no harness)
void set_intercept_relay(bool intercept) {
  UNUSED(intercept);
}

// ////////////////// White Panda //////////////////// //

void white_enable_can_transceiver(uint8_t transceiver, bool enabled) {
  switch (transceiver){
    case 1U:
      set_gpio_output(GPIOC, 1, !enabled);
      break;
    case 2U:
      set_gpio_output(GPIOC, 13, !enabled);
      break;
    case 3U:
      set_gpio_output(GPIOA, 0, !enabled);
      break;
    default:
      puts("Invalid CAN transceiver ("); puth(transceiver); puts("): enabling failed\n");
      break;
  }
}

void white_enable_can_transceivers(bool enabled) {
  uint8_t t1 = enabled ? 1U : 2U;  // leave transceiver 1 enabled to detect CAN ignition
  for(uint8_t i=t1; i<=3U; i++) {
    white_enable_can_transceiver(i, enabled);
  }
}

void white_set_led(uint8_t color, bool enabled) {
  switch (color){
    case LED_RED:
      set_gpio_output(GPIOC, 9, !enabled);
      break;
     case LED_GREEN:
      set_gpio_output(GPIOC, 7, !enabled);
      break;
    case LED_BLUE:
      set_gpio_output(GPIOC, 6, !enabled);
      break;
    default:
      break;
  }
}

void white_set_usb_power_mode(uint8_t mode){
  bool valid_mode = true;
  switch (mode) {
    case USB_POWER_CLIENT:
      // B2,A13: set client mode
      set_gpio_output(GPIOB, 2, 0);
      set_gpio_output(GPIOA, 13, 1);
      break;
    case USB_POWER_CDP:
      // B2,A13: set CDP mode
      set_gpio_output(GPIOB, 2, 1);
      set_gpio_output(GPIOA, 13, 1);
      break;
    case USB_POWER_DCP:
      // B2,A13: set DCP mode on the charger (breaks USB!)
      set_gpio_output(GPIOB, 2, 0);
      set_gpio_output(GPIOA, 13, 0);
      break;
    default:
      valid_mode = false;
      puts("Invalid usb power mode\n");
      break;
  }

  if (valid_mode) {
    usb_power_mode = mode;
  }
}

void white_set_gps_mode(uint8_t mode) {
  switch (mode) {
    case GPS_DISABLED:
      // ESP OFF
      set_gpio_output(GPIOC, 14, 0);
      set_gpio_output(GPIOC, 5, 0);
      break;
#ifndef EON
    case GPS_ENABLED:
      // ESP ON
      set_gpio_output(GPIOC, 14, 1);
      set_gpio_output(GPIOC, 5, 1);
      break;
#endif
    case GPS_BOOTMODE:
      set_gpio_output(GPIOC, 14, 1);
      set_gpio_output(GPIOC, 5, 0);
      break;
    default:
      puts("Invalid ESP/GPS mode\n");
      break;
  }
}

void white_set_can_mode(uint8_t mode){
  switch (mode) {
    case CAN_MODE_NORMAL:
      // B12,B13: disable GMLAN mode
      set_gpio_mode(GPIOB, 12, MODE_INPUT);
      set_gpio_mode(GPIOB, 13, MODE_INPUT);

      // B3,B4: disable GMLAN mode
      set_gpio_mode(GPIOB, 3, MODE_INPUT);
      set_gpio_mode(GPIOB, 4, MODE_INPUT);

      // B5,B6: normal CAN2 mode
      set_gpio_alternate(GPIOB, 5, GPIO_AF9_CAN2);
      set_gpio_alternate(GPIOB, 6, GPIO_AF9_CAN2);

      // A8,A15: normal CAN3 mode
      set_gpio_alternate(GPIOA, 8, GPIO_AF11_CAN3);
      set_gpio_alternate(GPIOA, 15, GPIO_AF11_CAN3);
      break;
    case CAN_MODE_GMLAN_CAN2:
      // B5,B6: disable CAN2 mode
      set_gpio_mode(GPIOB, 5, MODE_INPUT);
      set_gpio_mode(GPIOB, 6, MODE_INPUT);

      // B3,B4: disable GMLAN mode
      set_gpio_mode(GPIOB, 3, MODE_INPUT);
      set_gpio_mode(GPIOB, 4, MODE_INPUT);

      // B12,B13: GMLAN mode
      set_gpio_alternate(GPIOB, 12, GPIO_AF9_CAN2);
      set_gpio_alternate(GPIOB, 13, GPIO_AF9_CAN2);

      // A8,A15: normal CAN3 mode
      set_gpio_alternate(GPIOA, 8, GPIO_AF11_CAN3);
      set_gpio_alternate(GPIOA, 15, GPIO_AF11_CAN3);
      break;
    case CAN_MODE_GMLAN_CAN3:
      // A8,A15: disable CAN3 mode
      set_gpio_mode(GPIOA, 8, MODE_INPUT);
      set_gpio_mode(GPIOA, 15, MODE_INPUT);

      // B12,B13: disable GMLAN mode
      set_gpio_mode(GPIOB, 12, MODE_INPUT);
      set_gpio_mode(GPIOB, 13, MODE_INPUT);

      // B3,B4: GMLAN mode
      set_gpio_alternate(GPIOB, 3, GPIO_AF11_CAN3);
      set_gpio_alternate(GPIOB, 4, GPIO_AF11_CAN3);

      // B5,B6: normal CAN2 mode
      set_gpio_alternate(GPIOB, 5, GPIO_AF9_CAN2);
      set_gpio_alternate(GPIOB, 6, GPIO_AF9_CAN2);
      break;
    default:
      puts("Tried to set unsupported CAN mode: "); puth(mode); puts("\n");
      break;
  }
}

uint32_t white_read_current(void){
  return adc_get(ADCCHAN_CURRENT);
}

uint32_t marker = 0;
void white_usb_power_mode_tick(uint32_t uptime){

  // on EON or BOOTSTUB, no state machine
#if !defined(BOOTSTUB) && !defined(EON)
  #define CURRENT_THRESHOLD 0xF00U
  #define CLICKS 5U // 5 seconds to switch modes

  uint32_t current = white_read_current();

  // ~0x9a = 500 ma
  // puth(current); puts("\n");

  switch (usb_power_mode) {
    case USB_POWER_CLIENT:
      if ((uptime - marker) >= CLICKS) {
        if (!is_enumerated) {
          puts("USBP: didn't enumerate, switching to CDP mode\n");
          // switch to CDP
          white_set_usb_power_mode(USB_POWER_CDP);
          marker = uptime;
        }
      }
      // keep resetting the timer if it's enumerated
      if (is_enumerated) {
        marker = uptime;
      }
      break;
    case USB_POWER_CDP:
      // been CLICKS clicks since we switched to CDP
      if ((uptime - marker) >= CLICKS) {
        // measure current draw, if positive and no enumeration, switch to DCP
        if (!is_enumerated && (current < CURRENT_THRESHOLD)) {
          puts("USBP: no enumeration with current draw, switching to DCP mode\n");
          white_set_usb_power_mode(USB_POWER_DCP);
          marker = uptime;
        }
      }
      // keep resetting the timer if there's no current draw in CDP
      if (current >= CURRENT_THRESHOLD) {
        marker = uptime;
      }
      break;
    case USB_POWER_DCP:
      // been at least CLICKS clicks since we switched to DCP
      if ((uptime - marker) >= CLICKS) {
        // if no current draw, switch back to CDP
        if (current >= CURRENT_THRESHOLD) {
          puts("USBP: no current draw, switching back to CDP mode\n");
          white_set_usb_power_mode(USB_POWER_CDP);
          marker = uptime;
        }
      }
      // keep resetting the timer if there's current draw in DCP
      if (current < CURRENT_THRESHOLD) {
        marker = uptime;
      }
      break;
    default:
      puts("USB power mode invalid\n");  // set_usb_power_mode prevents assigning invalid values
      break;
  }
#else
  UNUSED(uptime);
#endif
}

void white_set_ir_power(uint8_t percentage){
  UNUSED(percentage);
}

void white_set_fan_power(uint8_t percentage){
  UNUSED(percentage);
}

bool white_check_ignition(void){
  // ignition is on PA1
  return !get_gpio_input(GPIOA, 1);
}

void white_set_phone_power(bool enabled){
  UNUSED(enabled);
}

void white_set_clock_source_mode(uint8_t mode){
  UNUSED(mode);
}

void white_set_siren(bool enabled){
  UNUSED(enabled);
}

void white_grey_common_init(void) {
  common_init_gpio();

  // C3: current sense
  set_gpio_mode(GPIOC, 3, MODE_ANALOG);

  // A1: started_alt
  set_gpio_pullup(GPIOA, 1, PULL_UP);

  // A2, A3: USART 2 for debugging
  set_gpio_alternate(GPIOA, 2, GPIO_AF7_USART2);
  set_gpio_alternate(GPIOA, 3, GPIO_AF7_USART2);

  // A4, A5, A6, A7: SPI
  set_gpio_alternate(GPIOA, 4, GPIO_AF5_SPI1);
  set_gpio_alternate(GPIOA, 5, GPIO_AF5_SPI1);
  set_gpio_alternate(GPIOA, 6, GPIO_AF5_SPI1);
  set_gpio_alternate(GPIOA, 7, GPIO_AF5_SPI1);

  // B12: GMLAN, ignition sense, pull up
  set_gpio_pullup(GPIOB, 12, PULL_UP);

  /* GMLAN mode pins:
      M0(B15)  M1(B14)  mode
      =======================
      0        0        sleep
      1        0        100kbit
      0        1        high voltage wakeup
      1        1        33kbit (normal)
  */
  set_gpio_output(GPIOB, 14, 1);
  set_gpio_output(GPIOB, 15, 1);

  // B7: K-line enable
  set_gpio_output(GPIOB, 7, 1);

  // C12, D2: Setup K-line (UART5)
  set_gpio_alternate(GPIOC, 12, GPIO_AF8_UART5);
  set_gpio_alternate(GPIOD, 2, GPIO_AF8_UART5);
  set_gpio_pullup(GPIOD, 2, PULL_UP);

  // L-line enable
  set_gpio_output(GPIOA, 14, 1);

  // C10, C11: L-Line setup (USART3)
  set_gpio_alternate(GPIOC, 10, GPIO_AF7_USART3);
  set_gpio_alternate(GPIOC, 11, GPIO_AF7_USART3);
  set_gpio_pullup(GPIOC, 11, PULL_UP);

  // Enable CAN transceivers
  white_enable_can_transceivers(true);

  // Disable LEDs
  white_set_led(LED_RED, false);
  white_set_led(LED_GREEN, false);
  white_set_led(LED_BLUE, false);

  // Set normal CAN mode
  white_set_can_mode(CAN_MODE_NORMAL);

  // Init usb power mode
  uint32_t voltage = adc_get_voltage();
  // Init in CDP mode only if panda is powered by 12V.
  // Otherwise a PC would not be able to flash a standalone panda with EON build
  if (voltage > 8000U) {  // 8V threshold
    white_set_usb_power_mode(USB_POWER_CDP);
  } else {
    white_set_usb_power_mode(USB_POWER_CLIENT);
  }
}

void white_init(void) {
  white_grey_common_init();

  // Set ESP off by default
  current_board->set_gps_mode(GPS_DISABLED);
}

const struct harness_configuration white_harness_config = {
  .has_harness = false
};

const struct board board_white = {
  .board_type = "White",
  .harness_config = &white_harness_config,
  .init = white_init,
  .enable_can_transceiver = white_enable_can_transceiver,
  .enable_can_transceivers = white_enable_can_transceivers,
  .set_led = white_set_led,
  .set_usb_power_mode = white_set_usb_power_mode,
  .set_gps_mode = white_set_gps_mode,
  .set_can_mode = white_set_can_mode,
  .usb_power_mode_tick = white_usb_power_mode_tick,
  .check_ignition = white_check_ignition,
  .read_current = white_read_current,
  .set_fan_power = white_set_fan_power,
  .set_ir_power = white_set_ir_power,
  .set_phone_power = white_set_phone_power,
  .set_clock_source_mode = white_set_clock_source_mode,
  .set_siren = white_set_siren
};

// //////////////// Board Detection ////////////////// //

void detect_board_type(void) {
  hw_type = HW_TYPE_WHITE_PANDA;
  current_board = &board_white;
}

bool has_external_debug_serial = 0;

void detect_configuration(void) {
  // detect if external serial debugging is present
  has_external_debug_serial = detect_with_pull(GPIOA, 3, PULL_DOWN);
}

// //////////////// Board Functions ////////////////// //

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
