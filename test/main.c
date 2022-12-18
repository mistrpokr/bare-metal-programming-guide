#include "mcu.h"

static volatile uint32_t s_ticks;
void
SysTick_Handler(void)
{
  s_ticks++;
}

int
main(void)
{
  systick_init(SYS_FREQ / 1000);
  uint32_t timer = 0, period = 250;

  uint16_t led_blue = PIN('B', 7);
  uint16_t led_red = PIN('B', 14);
  gpio_set_mode(led_blue, GPIO_MODE_OUTPUT);
  gpio_set_mode(led_red, GPIO_MODE_OUTPUT);

  uart_init(UART3, 115200);

  int ct = 0;
  int32_t ct_32 = 0;

  while (1) {
    if (timer_expired(&timer, period, s_ticks)) {
      static bool on;
      gpio_write(led_blue, on);
      on = !on;

      uart_write_buffer(UART3, "(write_buffer)\r\n", 4);
      printf("Hi! \r\n");
      printf("Counter = %d, Counter32 = %ld\r\n", ct++, ct_32++);
    }
  }

  return 0;
}
