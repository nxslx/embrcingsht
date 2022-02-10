/*
 * Since I don't use any of those cubeshmuby ides lets make a simple
 * makefile based project with libopencm3 in it.
 *
 * 1. stm32f103c8t6 is required (g0t one)
 *
 * 2. Read into uint8_t variable from a flash memory till MK starts.
 * The address to read is 0x0801AC00 - 0x0801AC01, then write 0xAA,0xEE
 * into 0x0801AC02 and 0x0801AC03 corespondingly.
 *
 * 3. Once a second send those values into (9600,8) uart port. UART1
 * 4. Receive values from UART2 and write them into comparator of PA8,
 * which has to be configured as a 5Khz PWM.
 *
 * 5. All the bytes received via UART2 have to be pushed into a 20bytes queue.
 *
 * 6. Use an external interrupt with falling edge which triggers a PB0 (12bit,
 * 12Mhz) comparator. The value has to be saved into a variable.
 *
 * 7. Check if PB2 is grounded, if so make it push into USART1 with +1 second delay
 *
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/cm3/nvic.h>
#include <stdint.h>
#include "nvm.h"

// {{{ initial settings
static void clock_setup(void) {
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
  rcc_periph_clock_enable(RCC_GPIOC);

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);

  rcc_periph_clock_enable(RCC_USART1);
  rcc_periph_clock_enable(RCC_USART2);
}

static void pwm_setup(void) {
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(RCC_TIM3);

  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO8);
  timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

  /*
   * Timer speed before prescaler = 72MHz
   * Lets keep it zero to make easier to check if
   * values values from USAR2 affect the ocr speed
   */
  timer_set_prescaler(TIM3, 0);

  /*
   * Supposed to be a 5Khz period
   * TODO: check if the assumption is correct
   */
  timer_set_period(TIM3, 5000);

  timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
  timer_enable_oc_preload(TIM3, TIM_OC1);
  timer_enable_preload(TIM3);

  timer_enable_oc_output(TIM3, TIM_OC1);

  timer_enable_counter(TIM3);
}

static void adc_setup(void) {
  nvic_set_priority(NVIC_ADC1_2_IRQ, 0);
  nvic_enable_irq(NVIC_ADC1_2_IRQ);

  /*
   * 6. There is a problem with such approach.
   * There is always a library for ADC for a very specific hardware which
   * I couldn't find for this one. All I can do now is rely on the:
   * https://github.com/libopencm3/libopencm3/blob/master/lib/stm32/f1/adc.c
   */
  rcc_periph_clock_enable(RCC_ADC1);
  adc_power_off(ADC1);

  adc_enable_scan_mode(ADC1);
  rcc_periph_reset_pulse(RST_ADC1);

  adc_enable_eoc_interrupt_injected(ADC1);

  gpio_set_mode(GPIOB, GPIO_CNF_INPUT_ANALOG, GPIO_CNF_INPUT_FLOAT, GPIO0);
  adc_power_on(ADC1);

  /*
   * I'm pretty sure here should be much more
   * adc specific stuff for stm32. Didn't used that much
   * on bluepill, have to improvise :)
   */
  for (int i = 0; i < 800000; i++)    /* Wait a bit. */
    __asm__("nop");

  adc_reset_calibration(ADC1);
  adc_calibrate(ADC1);
}

static void tim_setup(void) {
  rcc_periph_clock_enable(RCC_TIM2);
  nvic_enable_irq(NVIC_TIM2_IRQ);
  rcc_periph_reset_pulse(RST_TIM2);

  timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
    TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

  /*
   * Not sure, but looks like a mathematecally
   * correct division for a single second using 72mz clock.
   *
   * TODO: Make some tests to be sure this is precise enough
   */
  timer_set_prescaler(TIM2, 72000000UL/65536UL * 1000);

  timer_continuous_mode(TIM2);
  timer_set_period(TIM2, 1000);

  timer_enable_counter(TIM2);
  timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}

static void gpio_setup(void) {
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

  /* PB2 */
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
      GPIO_CNF_INPUT_FLOAT, GPIO2);

  /* UART1 */
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
          GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

  usart_set_baudrate(USART1, 9600);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

  usart_enable(USART1);

  /* UART2 */
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);

  usart_set_baudrate(USART2, 9600);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_mode(USART2, USART_MODE_TX);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

  USART_CR1(USART2) |= USART_CR1_RXNEIE;

  usart_enable(USART2);
}

// }}}

#define T2_READ_SECTOR_START 0x0801AC00
#define T2_WRITE_SECTOR_START 0x0801AC02
uint16_t flash_bytes;

int main(void) {
  /*
   * 2. Read and write NVM values.
   * Since we are using a 16bit values and sector could be read
   * by 2 bytes (size) we can easily use a 16bit variable as a regular buffer
   */
  uint16_t t2_wtmp = 0xaa;
  t2_wtmp <<= 8;
  t2_wtmp |= 0xee;

  readSector(T2_READ_SECTOR_START, &flash_bytes, 2);
  /*
   * TODO: Still didn't get the point, it should mean something, anything ?
   */
  writeSector(T2_WRITE_SECTOR_START, &t2_wtmp, 2);

  clock_setup();
  gpio_setup();
  tim_setup();
  pwm_setup();
  adc_setup();

  while (1)
    ;;;

  return 0;
}

volatile uint32_t adc1_value;
void adc1_2_isr(void) {
    ADC_SR(ADC1) &= ~ADC_SR_JEOC;
    adc1_value = adc_read_injected(ADC1,1);
}

#define QUEUE_SIZ 20
static void push_queue(uint8_t v) {
  static uint8_t q[QUEUE_SIZ];
  /*
   * A simple queue with shift which looks like this:
   * _ _ _ _ _
   * 1 | 2 | 3
   * ---------
   * if we push (4) into the q it will shift left like this:
   * _ _ _ _ _
   * 2 | 3 | 4
   * ---------
   */

  for (int i = 1; i < QUEUE_SIZ; i++)
    q[i-1] = q[i];

  q[QUEUE_SIZ-1] = v;
}

void usart2_isr(void) {
  uint8_t data = 0;

  if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
      ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {
    /*
     * 4. Set prescaller for TIM3 from
     *  the value received from USART2
     */
    data = usart_recv(USART2);
    timer_set_period(TIM3, data);

    /*
     * 5. Push the value into queue
     */
    push_queue(data);

    USART_CR1(USART2) |= USART_CR1_TXEIE;
  }
}

volatile uint8_t time_shift;
void tim2_isr(void) {
  if (timer_get_flag(TIM2, TIM_SR_CC1IF)) {
    timer_clear_flag(TIM2, TIM_SR_CC1IF);
    timer_set_oc_value(TIM2, TIM_OC1, 0);

    /*
     * 7. Make it slower twice when PB2 is pulled into the ground
     */
    if (gpio_get(GPIOB, GPIO2) && time_shift++ < 2)
      return;

    time_shift = 0;
    /*
     * 3. Send bytes read from a flash memory into USART1
     */
    usart_send_blocking(USART1, flash_bytes & 0xff);
    usart_send_blocking(USART1, flash_bytes << 8);

    /*
     * The mother of debug
     */
    gpio_toggle(GPIOC, GPIO13);
  }
}

/*
 * So far, I didn't realize where should I use those
 * flash read/writes, I suppose it is some kind of EEPROM replacement
 * for the bluepill using such a haxy way, I like this.
 *
 * The queue is probably used for logging, idk.
 *
 * The source code wasn't properly tested because it requires some hardware
 * setup which I hate to make every time for a simple tests, at a first look
 * with an oscilloscope and blinkywinky debug it works, probably it will
 * totally fail in a real life project and it definitely wasn't supposed to be such.
 */

