#include <inttypes.h>
#include <stdbool.h>

#define BIT(x) (1UL << (x)) // BIT(3) = 0000 1000
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

struct rcc
{
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
    RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
    RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR, AHB3LPENR,
    RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR, RESERVED6[2],
    SSCGR, PLLI2SCFGR;
};
#define RCC ((struct rcc*)0x40023800)

struct gpio
{
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};

#define GPIO(bank) ((struct gpio*)(0x40020000 + bank * 0x400))

enum
{
  GPIO_MODE_INPUT,
  GPIO_MODE_OUTPUT,
  GPIO_MODE_AF,
  GPIO_MODE_ANALOG
};

struct systick
{
  volatile uint32_t CTRL, LOAD, VAL, CALIB;
};

#define SYSTICK ((struct systick*)0xe000e010)

static inline void
systick_init(uint32_t ticks)
{
  if ((ticks - 1) > 0xffffff)
    return; // Systick timer is 24 bit
  SYSTICK->LOAD = ticks - 1;
  SYSTICK->VAL = 0;
  SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2); // Enable Systick
  RCC->APB2ENR |= BIT(14);                  // Enable SYSCFG
}

static volatile uint32_t s_ticks;
void
SysTick_Handler(void)
{
  s_ticks++;
}

bool
timer_expired(uint32_t* t, uint32_t period, uint32_t now)
{
  if (now + period < *t)
    *t = 0;
  if (*t == 0)
    *t = now + period;
  if (*t > now)
    return false;
  *t = (now - *t) > period ? now + period : *t + period;
  return true;
}

static inline void
gpio_set_mode(uint16_t pin, uint8_t mode)
{
  struct gpio* gpio = GPIO(PINBANK(pin));
  int n = PINNO(pin);
  gpio->MODER &= ~(3U << (n * 2)); // 3U = 0b11
  gpio->MODER |= (mode & 3U) << (n * 2);
}

static inline void
gpio_write(uint16_t pin, bool val)
{
  struct gpio* gpio = GPIO(PINBANK(pin));
  gpio->BSRR |= (1U << PINNO(pin)) << (val ? 0 : 16);
  // True: offset=0, write to BSy; False: offset=16, write to BRy
}

static inline void
spin(volatile uint32_t count)
{
  while (count--)
    asm("nop");
}

int
main(void)
{
  uint16_t led_blue = PIN('B', 7);
  uint16_t led_red = PIN('B', 14);
  RCC->AHB1ENR |= BIT(PINBANK(led_blue));
  RCC->AHB1ENR |= BIT(PINBANK(led_red));
  gpio_set_mode(led_blue, GPIO_MODE_OUTPUT);
  gpio_set_mode(led_red, GPIO_MODE_OUTPUT);

  systick_init(16000000 / 1000);
  uint32_t timer, period = 250;

  while (1) {
    if (timer_expired(&timer, period, s_ticks)) {
      static bool on;
      gpio_write(led_blue, on);
      on = !on;
    }

    // gpio_write(led_blue, true);
    // gpio_write(led_red, false);

    // gpio_write(led_blue, false);
    // gpio_write(led_red, true);
  }

  return 0;
}

__attribute__((naked, noreturn)) void
_reset(void)
{
  asm("ldr sp, = _estack");

  extern long _sbss, _ebss, _sdata, _edata, _sidata;

  /* Set bss region to 0 */
  for (long* src = &_sbss; src < &_ebss; src++)
    *src = 0;
  /* Copy from _sdata to _sidata */
  for (long *src = &_sdata, *dst = &_sidata; src < &_edata;)
    *src++ = *dst++;

  main();
  while (1)
    (void)0;
}

__attribute__((section(".vectors"))) void (*tab[16 + 91])(void) = {
  0, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler
};
