#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

static void clock_setup(void) {
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
  rcc_periph_clock_enable(RCC_GPIOC);
}

static void gpio_setup(void) {
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

int main(void) {
  int i;

  clock_setup();
  gpio_setup();

  /* Blink the LED (PC12) on the board. */
  while (1) {
    gpio_toggle(GPIOC, GPIO13);	/* LED on/off */
    for (i = 0; i < 8000000; i++)	/* Wait a bit. */
      __asm__("nop");
  }

  return 0;
}
